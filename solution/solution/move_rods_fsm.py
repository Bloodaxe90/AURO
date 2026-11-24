from typing import Callable, Generator
from week_4.robochart_fsm import Clock, Const, FINAL, Input, Operation, Param, Required, Scope, \
    State, StateMachine

from solution.power_maintainer import next_rod_move


CARRY_CRANE_Z = -4.0 # when carrying a rod to clear all cells
LOWERED_CRANE_Z = -6.38 # to touch the rod before attachment
EMPTY_TRAVEL_CRANE_Z = -3.0 # to prevent crane colliding with sides of canal when empty
IDENTIFY_POOL_CRANE_Z = -0.1 # crane fully raised

CRANE_WIDTH = 0.5
WALL_CRANE_SAFETY_MARGIN = 0.01 # always keep this far from pool walls and dividers when close to pool rim
                                # Must be less than 0.025 since the cells give the rods this much play (0.4 - 0.35) / 2 = 0.025

CANAL_Y = 0.0 # the cannal bit between reator and storage

POOL_CENTERS = {'reactor': -5.75, 'storage': 5.75}
POOL_X = {'reactor': (-9.0, -2.5), 'storage': (2.5, 9.0)}
POOL_Y = (-2.25, 2.25)

def apply_crane_wall_safety(x, y):
    # Used to keep the crane inside the wall as it was colliding with the wall when moving for cells along it (is only applied for cells close to the wall)
    bottom_x, top_x = POOL_X['reactor'] if x < 0 else POOL_X['storage']
    safety_margin = CRANE_WIDTH / 2.0 + WALL_CRANE_SAFETY_MARGIN
    return (min(max(x, bottom_x + safety_margin), top_x - safety_margin),
            min(max(y, POOL_Y[0] + safety_margin), POOL_Y[1] - safety_margin))


class CranePlatform(Scope):
    send_target: Callable[[float, float, float], None] # publish [x, y, z] on /crane/position_commands
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

    # True on /crane/at_target and cleared at the end of each cycle if not used
    at_target: Input[None] = Input(keep='cycle')

class MoveTo(Operation):
    x: Param[float]
    y: Param[float]
    z: Param[float]

    send_target: Required[Callable[[float, float, float], None]]
    at_target: Required[Input[None]]

    def initial(self):
        return self.Moving

    class Moving(State):

        def entry(self):
            self.send_target(self.x, self.y, self.z)
            # Give crane_controller time to get new_target
            yield from self.wait(1)

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

    # robot platform
    send_target: Required[Callable[[float, float, float], None]]
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
            for pool, centre_x in POOL_CENTERS.items():
                yield from self.move_to(centre_x, CANAL_Y, IDENTIFY_POOL_CRANE_Z)
                cells = yield from self.identify_pool(pool, centre_x, CANAL_Y, IDENTIFY_POOL_CRANE_Z)
                self.cells = {**self.cells, **cells}
                self.logger.info(f'{pool.capitalize()} pool cells identified')

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
                                 self.full_fuel_rod_power, self.storage_rod_level)

            if move is None:
                self.logger.info(f'Done after {self.move_counter} moves: power {current_power:.2f}%, '
                                 f'target {target_power:.2f}%')
                return FINAL
            def action():
                self.source_cell, self.target_cell, reason = move
                self.move_counter = self.move_counter + 1
                self.logger.info(f'Power {current_power:.2f}% target {target_power:.2f}%: {reason}, moving '
                                 f'{self.cells[self.source_cell]} rod {self.source_cell} -> '
                                 f'{self.target_cell}')
            return self.GetSource, action

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
                x, y = self.detach_crane_position()
                yield from self.move_to(x, y, CARRY_CRANE_Z)
            else:
                x, y = self.attach_crane_position()
                yield from self.move_to(x, y, EMPTY_TRAVEL_CRANE_Z)

        def transitions(self):
            return self.LowerToCell

    class LowerToCell(State):

        def entry(self):
            if self.carrying:
                x, y = self.detach_crane_position()
            else:
                x, y = self.attach_crane_position()
            yield from self.move_to(x, y, LOWERED_CRANE_Z)

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
                x, y = self.attach_crane_position()
                yield from self.move_to(x, y, CARRY_CRANE_Z)
            else:
                x, y = self.detach_crane_position()
                yield from self.move_to(x, y, EMPTY_TRAVEL_CRANE_Z)

        def transitions(self):
            if self.carrying:
                return self.MoveToCanal
            self.logger.info(f'Moved rod from {self.source_cell} to {self.target_cell}')
            return self.WaitForPowerUpdate

    class WaitForPowerUpdate(State):

        def entry(self):
            # as /current_power_level is published once a second wait for it to be updated with the new move
            yield from self.wait(2)

        def transitions(self):
            return self.NextRodToMove

    class MoveToCanal(State):

        def entry(self):
            x, _ = apply_crane_wall_safety(self.source_cell_x, self.source_cell_y)
            yield from self.move_to(x, CANAL_Y, CARRY_CRANE_Z)

        def transitions(self):
            return self.MoveAlongCanal

    class MoveAlongCanal(State):

        def entry(self):
            x, _ = apply_crane_wall_safety(self.target_cell_x, self.target_cell_y)
            yield from self.move_to(x, CANAL_Y, CARRY_CRANE_Z)

        def transitions(self):
            return self.MoveAboveCell
            
    # MoveAboveCell -> LowerToCell

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
                    
                    self.carrying = False
                    self.set_carrying(False)
                return self.RaiseFromCell, action
            self.logger.error('Failed to detach rod')
            return self.DetachRod

    # RaiseFromCell -> WaitForPowerUpdate -> NextRodToMove -> LOOP TILL NO RODS TO MOVE -> FINAL