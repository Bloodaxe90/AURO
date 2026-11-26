from typing import Callable, Generator
from week_4.robochart_fsm import Clock, Const, FINAL, Input, Operation, Param, Required, Scope, \
    State, StateMachine
from solution.task2.power_maintainer import MAX_POWER, MIN_TRIM, new_target_cell, \
    in_reactor, next_rod_move, rods_to_read
from solution.task2.pool_helper import CANAL_Y, apply_crane_wall_safety, route, POOL_CENTERS, TOTAL_CONTROL_RODS


CARRY_CRANE_Z = -4.0 # when carrying a rod to clear all cells
LOWERED_CRANE_Z = -6.38 # to touch the rod before attachment
IDENTIFY_POOL_CRANE_Z = -0.1 # crane fully raised
HOVER_CRANE_Z = -6.0

EXACT_TOLERANCE = 0.0 # uses the position and velocity tolerance values in crane_controller.py
READ_WAYPOINT_TOLERANCE = 0.03 # must be kept low to ensure crane can still read rod levels (extra issue at wall corners)
WAYPOINT_TOLERANCE = 0.2 # must still be kept relatively low to allow a raise somewhat before moving after detaching (so not scaping rods)

MOVE_TIMEOUT = 600 # Max time allowed to move between cells (60s)
LOWER_TIMEOUT = 100 # Max time allowed to lower into a cell (10s)

MAX_LOWER_ATTEMPTS = 3 # Max tried before giving up lowering to a cell
MAX_SERVICE_ATTEMPTS = 5 # Max service call attempts before giving up on a cell
MAX_ABANDONED_RODS = 3 # rods in a row that cannot be attached before the winch is assumed faulty
MAX_POOL_IDENTIFICATION_ATTEMPTS = 3 # Max pool identifications attempts in a row before the camera is assumed faulty

class CranePlatform(Scope):
    send_target: Callable[[float, float, float, float], None] # publish [x, y, z, tolerance] on /crane/position_commands
    set_carrying: Callable[[bool], None] # publish on /crane/carrying
    release_brake: Callable[[], Generator] # /crane/winch/brake/release
    engage_brake: Callable[[], Generator] # /crane/winch/brake/engage
    attach: Callable[[], Generator] # /crane/winch/attach
    detach: Callable[[], Generator] # /crane/winch/detach
    cell_pose: Callable[[str], Generator] # /cell_pose
    crane_pose: Callable[[], tuple] # /crane/joint_states
    identify_pool: Callable[[str, float, float, float], Generator]
    current_power: Callable[[], float] # /current_power_level
    target_power: Callable[[], float] # /target_power_level
    rod_level: Callable[[], Generator] # /crane/winch/rod_power_level

    # True on /crane/at_target and cleared at the end of each cycle if not used
    at_target: Input[None] = Input(keep='cycle')

class MoveTo(Operation):
    x: Param[float]
    y: Param[float]
    z: Param[float]
    tolerance: Param[float]
    timeout: Param[int]

    send_target: Required[Callable[[float, float, float, float], None]]
    at_target: Required[Input[None]]
    arrived: Required[bool]

    C: Clock

    def initial(self):
        return self.Moving

    class Moving(State):

        def entry(self):
            self.C.reset()
            self.send_target(self.x, self.y, self.z, self.tolerance)
            # Give crane_controller time to get new_target
            yield from self.wait(2)

        def transitions(self):
            if self.at_target.triggered():
                self.at_target.take()
                self.arrived = True
                return FINAL
            if self.C.since() >= self.timeout:
                self.logger.warning(f'Not at ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) after '
                                    f'{self.timeout / 10:.0f} s')
                self.arrived = False
                return FINAL

class MoveRods(StateMachine):

    control_rod_coefficient: Const[float] # power multiplier per control rod removed
    full_fuel_rod_power: Const[float] # % of maximum power for one fuel rod at level 100
    storage_rod_level: Const[float] # level of the storage rods in storage pool

    source_cell: str = '' # cell of rod to move
    target_cell: str = '' # cell to move rod to
    move_counter: int = 0 # how many moves have been started

    source_cell_x: float = 0.0
    source_cell_y: float = 0.0
    target_cell_x: float = 0.0
    target_cell_y: float = 0.0
    attach_offset_x: float = 0.0 # x offset of rod from cranes center when attached
    attach_offset_y: float = 0.0 # y offset of rod from cranes center when attached

    cell_found: bool = False # if cell_pose service call found the cell
    succeeded: bool = False # if last service call
    carrying: bool = False # if the crane holds the rod
    arrived: bool = False # if the last move_to got there in time
    pool_identified: bool = False # if the last pool identification found every control rod

    attempts: int = 0 # tries at lowering
    abandoned_rods: int = 0 # rods given up on in a row

    cells: dict[str, str] = {} # map of every cell in the pool and its type 'fuel', 'control', 'empty'
    rod_offsets: dict[str, list[float]] = {} # where placed rods sit in their cells (initial ones are assumed to be centered)
    rod_levels: dict[str, float] = {} # known levels of fuel rods by the cell they are in
    cells_to_read: list[str] = [] # cells of the fuel rods to read the level of next
    read_cells: set[str] = set() # cells of the fuel rods that have been read to avoid reading the same rod twice

    # robot platform
    send_target: Required[Callable[[float, float, float, float], None]]
    set_carrying: Required[Callable[[bool], None]]
    release_brake: Required[Callable[[], Generator]]
    engage_brake: Required[Callable[[], Generator]]
    attach: Required[Callable[[], Generator]]
    detach: Required[Callable[[], Generator]]
    cell_pose: Required[Callable[[str], Generator] ]
    crane_pose: Required[Callable[[], tuple]]
    identify_pool: Required[Callable[[str, float, float, float], Generator]]
    current_power: Required[Callable[[], float]]
    target_power: Required[Callable[[], float]]
    rod_level: Required[Callable[[], Generator]]

    at_target: Required[Input[None]]

    # operation
    move_to: type[Operation] = MoveTo

    def source_rod(self):
        # Where the rod to pick up is (adding offset incase it was placed off-center by the crane)
        offset_x, offset_y = self.rod_offsets.get(self.source_cell, [0.0, 0.0])
        return self.source_cell_x + offset_x, self.source_cell_y + offset_y

    def attach_crane_position(self):
        # Try attach the rod from the cell so it sits centers on the crane (adding offset incase it was detached off center by the crane (due to wall safety or tolerance))
        return apply_crane_wall_safety(*self.source_rod())

    def detach_crane_position(self):
        # Try detach rod from the crane so it sits at the cells center (removing offset incase it was attached off center by the crane (due to wall safety or tolerance))
        return apply_crane_wall_safety(self.target_cell_x - self.attach_offset_x,
                                       self.target_cell_y - self.attach_offset_y)

    def initial(self):
        return self.ReleaseBrake

    class ReleaseBrake(State):

        def entry(self):
            self.attempts = self.attempts + 1
            self.succeeded = yield from self.release_brake()

        def transitions(self):
            if self.succeeded:
                def action():
                    self.attempts = 0
                return self.IdentifyPools, action
            if self.attempts < MAX_SERVICE_ATTEMPTS:
                self.logger.warning(f'Failed to release brake (try {self.attempts}), trying again')
                return self.ReleaseBrake
            self.logger.error(f'Failed to release brake after {MAX_SERVICE_ATTEMPTS} attempts, giving up')
            return self.Fault

    class IdentifyPools(State):

        def entry(self):
            self.attempts = self.attempts + 1
            self.pool_identified = False   
            x, y = self.crane_pose()
            yield from self.move_to(x, y, IDENTIFY_POOL_CRANE_Z, WAYPOINT_TOLERANCE, MOVE_TIMEOUT)
            if not self.arrived:
                return
            
            # storage first so the crane finishes over the reactor as the first move usually takes out a control rod first
            for pool in ('storage', 'reactor'):
                centre_x = POOL_CENTERS[pool]
                yield from self.move_to(centre_x, CANAL_Y, IDENTIFY_POOL_CRANE_Z, EXACT_TOLERANCE, MOVE_TIMEOUT)
                if not self.arrived:
                    return
                cells = yield from self.identify_pool(pool, centre_x, CANAL_Y, IDENTIFY_POOL_CRANE_Z)
                if cells is None:
                    return # no images from the camera   
                self.cells = {**self.cells, **cells}
                self.logger.info(f'{pool.capitalize()} pool cells identified')

            # every control rod must be found
            self.pool_identified = list(self.cells.values()).count('control') == TOTAL_CONTROL_RODS
            if not self.pool_identified:
                return   

            storage_levels = {cell: self.storage_rod_level for cell, rod in self.cells.items()
                              if not in_reactor(cell) and rod == 'fuel'}
            self.rod_levels = {**storage_levels, **self.rod_levels}

        def transitions(self):
            if not self.arrived:
                return self.Fault

            if not self.pool_identified:
                if self.attempts < MAX_POOL_IDENTIFICATION_ATTEMPTS:
                    self.logger.warning(f'Failed to identify pool (try {self.attempts}), trying again')
                    return self.IdentifyPools
                self.logger.error(f'Failed to identify pool after {MAX_POOL_IDENTIFICATION_ATTEMPTS} times, giving up')
                return self.Fault

            cell_types = {cell_type: list(self.cells.values()).count(cell_type)
                      for cell_type in ('fuel', 'control', 'empty')}
            self.logger.info(f'Pool Cell Counts: {cell_types}')
            return self.NextRodToMove

    class NextRodToMove(State):

        def entry(self):
            yield # so on_chagne logs state

        def transitions(self):
            current_power, target_power = self.current_power(), self.target_power()

            if current_power is None or target_power is None:
                return None # wait for the reactor monitor to publish the power levels

            move = next_rod_move(self.cells, current_power, target_power,self.control_rod_coefficient,
                                 self.full_fuel_rod_power, self.storage_rod_level, self.rod_levels)

            if move is not None:
                def action():
                    self.source_cell, self.target_cell, reason = move
                    self.move_counter = self.move_counter + 1
                    self.logger.info(f'Power {current_power:.2f}% target {target_power:.2f}%: '
                                     f'{reason}, moving {self.cells[self.source_cell]} rod '
                                     f'{self.source_cell} -> {self.target_cell}')
                return self.GetSource, action

            # no move worth making with the levels known: read some more, if worth it
            reads = rods_to_read(self.cells, self.rod_levels, self.read_cells, current_power, target_power,
                                   *self.crane_pose())
            if reads:
                def action():
                    self.cells_to_read = reads
                    self.logger.info(f'Power {current_power:.2f}% target {target_power:.2f}%: '
                                     f'reading the levels of {", ".join(reads)}')
                return self.ReadRodLevels, action

            waste = current_power - target_power
            if waste > MIN_TRIM:
                # rods_to_read stops early when every reactor rod has been read
                self.logger.info(f'All {len(self.read_cells)} reactor rod read, no rod level small enough to '
                                 f'trim the remaining {waste:.2f}% waste')

            self.logger.info(f'Target reached after {self.move_counter} moves: power '
                             f'{current_power:.2f}%, target {target_power:.2f}%')
            return self.MonitorPower

    class GetSource(State):

        def entry(self):
            self.cell_found, self.source_cell_x, self.source_cell_y = yield from self.cell_pose(self.source_cell)

        def transitions(self):
            if self.cell_found:
                return self.GetTarget
            self.logger.error(f'Failed to find cell {self.source_cell}')
            return self.NextRodToMove

    class GetTarget(State):

        def entry(self):
            self.cell_found, self.target_cell_x, self.target_cell_y = yield from self.cell_pose(self.target_cell)

        def transitions(self):
            if self.cell_found:
                return self.CheckValidRodMove
            self.logger.error(f'Failed to find cell {self.target_cell}')
            return self.NextRodToMove

    class CheckValidRodMove(State):

        def entry(self):
            yield # so on_chagne logs state

        def transitions(self):
            if self.cells[self.source_cell] == 'empty':
                self.logger.error(f'Failed to find rod in cell {self.source_cell}')
                return self.NextRodToMove
            if self.cells[self.target_cell] != 'empty':
                self.logger.error(f'Failed to move rod to cell {self.target_cell} as it is not empty')
                return self.NextRodToMove
            return self.MoveAboveCell

    class MoveAboveCell(State):

        def entry(self):
            if self.carrying:
                x, y = self.detach_crane_position()
                z = CARRY_CRANE_Z
            else:
                x, y = self.attach_crane_position()
                z = HOVER_CRANE_Z

            crane_x, _ = self.crane_pose()
            waypoints = route(crane_x, x, y, WAYPOINT_TOLERANCE)
            for i, (waypoint_x, waypoint_y) in enumerate(waypoints):
                # pass through the canal waypoints but stop exactly above cell
                last = i == len(waypoints) - 1
                yield from self.move_to(waypoint_x, waypoint_y, z,
                                        EXACT_TOLERANCE if last else WAYPOINT_TOLERANCE, MOVE_TIMEOUT)
                if not self.arrived:
                    return

        def transitions(self):
            if not self.arrived:
                return self.Fault

            def action():
                self.attempts = 0
            return self.LowerToCell, action

    class LowerToCell(State):

        def entry(self):
            self.attempts = self.attempts + 1
            if self.carrying:
                x, y = self.detach_crane_position()
            else:
                x, y = self.attach_crane_position()
            yield from self.move_to(x, y, LOWERED_CRANE_Z, EXACT_TOLERANCE, LOWER_TIMEOUT)

        def transitions(self):
            if self.arrived:
                def action():
                    self.attempts = 0
                if self.carrying:
                    return self.DetachRod, action
                return self.AttachRod, action

            failed_cell = self.target_cell if self.carrying else self.source_cell
            if self.attempts < MAX_LOWER_ATTEMPTS:
                self.logger.warning(f'Failed to lower to cell {failed_cell} (try {self.attempts}), '
                                    'trying again')
                return self.RetryLower

            # something is in the way so cell map is wrong
            self.logger.warning(f'Failed to lower to cell {failed_cell} after {MAX_LOWER_ATTEMPTS} attempts, giving up')

            # setting failed cell to unknown so it can be re identified next time
            def action():
                self.cells = {**self.cells, failed_cell: 'unknown'}
            return self.AbandonCell, action

    class RetryLower(State):

        def entry(self):
            # straight back up to setup for lowering again
            if self.carrying:
                (x, y), z = self.detach_crane_position(), CARRY_CRANE_Z
            else:
                (x, y), z = self.attach_crane_position(), HOVER_CRANE_Z
            yield from self.move_to(x, y, z, EXACT_TOLERANCE, MOVE_TIMEOUT)

        def transitions(self):
            if not self.arrived:
                return self.Fault
            return self.LowerToCell

    class AbandonCell(State):

        def entry(self):
            # straight back up
            x, y = self.crane_pose()
            yield from self.move_to(x, y, CARRY_CRANE_Z if self.carrying else HOVER_CRANE_Z,
                                    WAYPOINT_TOLERANCE, MOVE_TIMEOUT)

        def transitions(self):
            if not self.arrived:
                return self.Fault
            if not self.carrying:
                return self.NextRodToMove

            # if still holding the rod take it to another empty cell in the same pool
            target_cell = new_target_cell(self.cells, self.target_cell)
            if target_cell is None:
                self.logger.error(f'No empty cells for rod from {self.source_cell}')
                return self.Fault

            def action():
                self.logger.info(f'Plan changed moving rod to {target_cell}')
                self.target_cell = target_cell
            return self.GetTarget, action

    class AttachRod(State):

        def entry(self):
            self.attempts = self.attempts + 1
            self.succeeded = yield from self.attach()

        def transitions(self):
            if self.succeeded:
                def action():
                    rod_x, rod_y = self.source_rod()
                    crane_x, crane_y = self.crane_pose()
                    self.attach_offset_x = rod_x - crane_x
                    self.attach_offset_y = rod_y - crane_y
                    self.carrying = True
                    self.set_carrying(True)
                    self.abandoned_rods = 0 
                return self.RaiseFromCell, action

            if self.attempts < MAX_SERVICE_ATTEMPTS:
                self.logger.warning(f'Failed to attach rod at cell {self.source_cell} (try {self.attempts}), trying again')
                return self.AttachRod

            # no rod there so cell map is wrong
            self.logger.warning(f'Failed to attach rod at cell {self.source_cell} after {MAX_SERVICE_ATTEMPTS} attempts, giving up')

            if self.abandoned_rods + 1 >= MAX_ABANDONED_RODS:
                self.logger.error(f'Failed to attach rods to {MAX_ABANDONED_RODS} cells in a row, giving up')
                return self.Fault

            # setting failed cell to unknown so it can be re identified next time
            def action():
                self.cells = {**self.cells, self.source_cell: 'unknown'}
                self.abandoned_rods = self.abandoned_rods + 1
            return self.AbandonCell, action

    class RaiseFromCell(State):

        def entry(self):
            if self.carrying:
                x, y = self.attach_crane_position()
                z = CARRY_CRANE_Z
            else:
                x, y = self.detach_crane_position()
                z = HOVER_CRANE_Z
            yield from self.move_to(x, y, z, WAYPOINT_TOLERANCE, MOVE_TIMEOUT)

        def transitions(self):
            if not self.arrived:
                return self.Fault

            if self.carrying:
                return self.MoveAboveCell
            self.logger.info(f'Moved rod from {self.source_cell} to {self.target_cell}')
            return self.WaitForPowerUpdate

    class WaitForPowerUpdate(State):

        def entry(self):
            # as /current_power_level is published once a second wait for it to be updated with the new move
            yield from self.wait(12)

        def transitions(self):
            return self.NextRodToMove

    class DetachRod(State):

        def entry(self):
            self.attempts = self.attempts + 1
            self.succeeded = yield from self.detach()

        def transitions(self):
            if self.succeeded:
                def action():
                    # Where the rod was let go (cranes position plus the held offset)
                    crane_x, crane_y = self.crane_pose()
                    offset = [crane_x + self.attach_offset_x - self.target_cell_x,
                              crane_y + self.attach_offset_y - self.target_cell_y]
                    self.logger.info(f'Rod placed {offset[0] * 1000:.2f} mm, '
                                     f'{offset[1] * 1000:.2f} mm (x, y) from the centre of '
                                     f'{self.target_cell}')
                    # update cell map and where the rod offset in the new cell
                    self.cells = {**self.cells, self.target_cell: self.cells[self.source_cell],
                                  self.source_cell: 'empty'}
                    offsets = {cell: offset for cell, offset in self.rod_offsets.items()
                               if cell != self.source_cell}
                    self.rod_offsets = {**offsets, self.target_cell: offset}

                    # ensure rods level moves with too
                    if self.source_cell in self.rod_levels:
                        levels = {cell: level for cell, level in self.rod_levels.items()
                                  if cell != self.source_cell}
                        self.rod_levels = {**levels,
                                           self.target_cell: self.rod_levels[self.source_cell]}

                    self.carrying = False
                    self.set_carrying(False)
                return self.RaiseFromCell, action

            if self.attempts < MAX_SERVICE_ATTEMPTS:
                self.logger.warning(f'Failed to detach rod (try {self.attempts}), trying again')
                return self.DetachRod

            self.logger.error(f'Failed to detach rod after {MAX_SERVICE_ATTEMPTS} attempts, giving up')
            return self.Fault

    class ReadRodLevels(State):

        def entry(self):
            for cell in self.cells_to_read:
                found, cell_x, cell_y = yield from self.cell_pose(cell)
                # passed through within READ_WAYPOINT_TOLERANCE so kept further from walls
                x, y = apply_crane_wall_safety(cell_x, cell_y, READ_WAYPOINT_TOLERANCE)
                # hovering just above the rods
                crane_x, _ = self.crane_pose()
                waypoints = route(crane_x, x, y, WAYPOINT_TOLERANCE)
                for i, (waypoint_x, waypoint_y) in enumerate(waypoints):

                    # pass through any canal waypoints but use a smaller tolerance for rod level reads
                    last = i == len(waypoints) - 1
                    yield from self.move_to(waypoint_x, waypoint_y, HOVER_CRANE_Z,
                                            READ_WAYPOINT_TOLERANCE if last else WAYPOINT_TOLERANCE,
                                            MOVE_TIMEOUT)
                    if not self.arrived:
                        return

                self.read_cells = {*self.read_cells, cell}  # counted even if the read fails
                found, level = yield from self.rod_level()
                if not found:
                    self.logger.warning(f'Failed to read fuel rod level in cell {cell}')
                    level = 0.0
                self.rod_levels = {**self.rod_levels, cell: level}
                self.logger.info(f'Fuel rod in {cell} has level {level:.2f}')

        def transitions(self):
            if not self.arrived:
                return self.Fault
            return self.NextRodToMove

    class Fault(State):

        def entry(self):
            # if catastrophic failure stop the crane so it doesnt keep moving
            self.logger.error('Major fault, engaging brake and stopping crane, human intervention needed')
            yield from self.engage_brake()

        def transitions(self):
            return FINAL

    class MonitorPower(State):

        def entry(self):
            # check the power again every few seconds
            yield from self.wait(50)

        def transitions(self):
            current_power, target_power = self.current_power(), self.target_power()
            if current_power < target_power or current_power > MAX_POWER:
                self.logger.warning(f'Power {current_power:.2f}% out of range, replanning')
                def action():
                    self.attempts = 0
                return self.IdentifyPools, action
            return self.MonitorPower