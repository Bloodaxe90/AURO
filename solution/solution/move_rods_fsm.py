from typing import Callable, Generator
from week_4.robochart_fsm import Clock, Const, FINAL, Input, Operation, Param, Required, Scope, \
    State, StateMachine
from solution.power_maintainer import MAX_POWER, in_reactor, next_rod_move, rods_to_read
from solution.pool_helper import CANAL_Y, apply_crane_wall_safety, route, POOL_CENTERS


CARRY_CRANE_Z = -4.0 # when carrying a rod to clear all cells
LOWERED_CRANE_Z = -6.38 # to touch the rod before attachment
IDENTIFY_POOL_CRANE_Z = -0.1 # crane fully raised
HOVER_CRANE_Z = -6.05

EXACT_TOLERANCE = 0.0 # uses the position and velocity tolerance values in crane_controller.py
WAYPOINT_TOLERANCE = 0.05

class CranePlatform(Scope):
    send_target: Callable[[float, float, float, float], None] # publish [x, y, z] on /crane/position_commands
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

    send_target: Required[Callable[[float, float, float, float], None]]
    at_target: Required[Input[None]]

    def initial(self):
        return self.Moving

    class Moving(State):

        def entry(self):
            self.send_target(self.x, self.y, self.z, self.tolerance)
            # Give crane_controller time to get new_target
            yield from self.wait(2)

        def transitions(self):
            if self.at_target.triggered():
                self.at_target.take()
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

    cells: dict[str, str] = {} # map of every cell in the pool and its type 'fuel', 'control', 'empty'
    rod_offsets: dict[str, list[float]] = {} # where placed rods sit in their cells (initial ones are assumed to be centered)
    rod_levels: dict[str, float] = {} # known levels of fuel rods by the cell they are in
    read_cells: list[str] = [] # cells of the fuel rods to read the level of next

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
    rod_level: Required[Callable[[], float]]

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
            self.succeeded = yield from self.release_brake()

        def transitions(self):
            if self.succeeded:
                return self.IdentifyPools
            self.logger.error('Failed to release brake')
            return self.ReleaseBrake

    class IdentifyPools(State):

        def entry(self):
            x, y = self.crane_pose()
            yield from self.move_to(x, y, IDENTIFY_POOL_CRANE_Z, WAYPOINT_TOLERANCE)
            for pool, centre_x in POOL_CENTERS.items():
                yield from self.move_to(centre_x, CANAL_Y, IDENTIFY_POOL_CRANE_Z, EXACT_TOLERANCE)
                cells = yield from self.identify_pool(pool, centre_x, CANAL_Y, IDENTIFY_POOL_CRANE_Z)
                self.cells = {**self.cells, **cells}
                self.logger.info(f'{pool.capitalize()} pool cells identified')

            storage_levels = {cell: self.storage_rod_level for cell, rod in self.cells.items()
                              if not in_reactor(cell) and rod == 'fuel'}
            self.rod_levels = {**storage_levels, **self.rod_levels}

            # hover over cells
            x, y = self.crane_pose()
            yield from self.move_to(x, y, HOVER_CRANE_Z, WAYPOINT_TOLERANCE)

        def transitions(self):
            cell_types = {cell_type: list(self.cells.values()).count(cell_type)
                      for cell_type in ('fuel', 'control', 'empty')}
            self.logger.info(f'Pool Cell Counts: {cell_types}')
            return self.NextRodToMove

    class NextRodToMove(State):

        def entry(self):
            pass

        def transitions(self):
            current_power, target_power = self.current_power(), self.target_power()

            if current_power is None or target_power is None:
                return None # wait for the reactor monitor to publish the power levels

            move = next_rod_move(self.cells, current_power, target_power, self.control_rod_coefficient,
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
            reads = rods_to_read(self.cells, self.rod_levels, current_power, target_power,
                                   *self.crane_pose())
            if reads:
                def action():
                    self.read_cells = reads
                    self.logger.info(f'Power {current_power:.2f}% target {target_power:.2f}%: '
                                     f'reading the levels of {", ".join(reads)}')
                return self.ReadRodLevels, action

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
                x, y, z = self.detach_crane_position(), CARRY_CRANE_Z
            else:
                x, y, z = self.attach_crane_position(), HOVER_CRANE_Z

            crane_x, _ = self.crane_pose()
            waypoints = route(crane_x, x, y, WAYPOINT_TOLERANCE)
            for i, (waypoint_x, waypoint_y) in enumerate(waypoints):
                # pass through the canal waypoints but stop exactly above cell
                last = i == len(waypoints) - 1
                yield from self.move_to(waypoint_x, waypoint_y, z,
                                        EXACT if last else WAYPOINT_TOLERANCE)

        def transitions(self):
            return self.LowerToCell

    class LowerToCell(State):

        def entry(self):
            if self.carrying:
                x, y = self.detach_crane_position()
            else:
                x, y = self.attach_crane_position()
            yield from self.move_to(x, y, LOWERED_CRANE_Z, EXACT_TOLERANCE)

        def transitions(self):
            if self.carrying:
                return self.DetachRod
            return self.AttachRod

    class AttachRod(State):

        def entry(self):
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
                return self.RaiseFromCell, action
            self.logger.error('Failed to attach rod')
            return self.AttachRod

    class RaiseFromCell(State):

        def entry(self):
            if self.carrying:
                x, y, z = self.attach_crane_position(), CARRY_CRANE_Z
            else:
                x, y, z = self.detach_crane_position(), HOVER_CRANE_Z
            yield from self.move_to(x, y, z, WAYPOINT_TOLERANCE)

        def transitions(self):
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
            self.logger.error('Failed to detach rod')
            return self.DetachRod

    class ReadRodLevels(State):

        def entry(self):
            for cell in self.read_cells:
                found, cell_x, cell_y = yield from self.cell_pose(cell)
                # passed through within WAYPOINT_TOLERANCE so kept further from walls
                x, y = apply_crane_wall_safety(cell_x, cell_y, WAYPOINT_TOLERANCE)
                # hovering just above the rods
                crane_x, _ = self.crane_pose()
                for waypoint_x, waypoint_y in route(crane_x, x, y, WAYPOINT_TOLERANCE):
                    yield from self.move_to(waypoint_x, waypoint_y, HOVER_CRANE_Z,
                                            WAYPOINT_TOLERANCE)

                found, level = yield from self.rod_level()
                if not found:
                    self.logger.warning(f'No fuel rod level read in {cell}')
                    level = 0.0
                self.rod_levels = {**self.rod_levels, cell: level}
                self.logger.info(f'Fuel rod in {cell} has level {level:.2f}')

        def transitions(self):
            return self.NextRodToMove

    class MonitorPower(State):

        def entry(self):
            # check the power again every few seconds
            yield from self.wait(50)

        def transitions(self):
            current_power, target_power = self.current_power(), self.target_power()
            if current_power < target_power or current_power > MAX_POWER:
                self.logger.warning(f'Power {current_power:.2f}% out of range, replanning')
                return self.IdentifyPools
            return self.MonitorPower