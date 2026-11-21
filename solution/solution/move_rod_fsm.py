from typing import Callable, Generator
from week_4.robochart_fsm import Const, FINAL, Input, Operation, Param, Required, Scope, State, \
    StateMachine

CARRY_WINCH_Z = -4.0 # when carrying a rod to clear all cells
LOWERED_WINCH_Z = -6.38 # to touch the rod before attachment
EMPTY_TRAVEL_WINCH_Z = LOWERED_WINCH_Z / 2.0 # to prevent winch colliding with sides of canal when empty
IDENTIFY_POOL_WINCH_Z = 0.0 # winch fully raised

WINCH_WIDTH = 0.5
WALL_WINCH_SAFETY_MARGIN = 0.02 # always keep this far from pool walls and dividers when close to the rim

CANAL_Y = 0.0 # the cannal bit between reator and storage

POOL_CENTERS = {'reactor': -5.75, 'storage': 5.75}
POOL_X = {'reactor': (-9.0, -2.5), 'storage': (2.5, 9.0)}
POOL_Y = (-2.25, 2.25)


def apply_winch_wall_safety(x, y):
    # Used to keep the winch inside the wall as it was colliding with the wall when moving for cells along it
    bottom_x, top_x = POOL_X['reactor'] if x < 0 else POOL_X['storage']
    safety_margin = WINCH_WIDTH / 2.0 + WALL_WINCH_SAFETY_MARGIN
    return (min(max(x, bottom_x + safety_margin), top_x - safety_margin),
            min(max(y, POOL_Y[0] + safety_margin), POOL_Y[1] - safety_margin))

# TODO: WINCH WALL SAFETY MUST BE UNDONE FOR PLACEMENT

class CranePlatform(Scope):
    send_target: Callable[[float, float, float], None] # publish [x, y, z] on /crane/position_commands
    set_carrying: Callable[[bool], None] # publish on /crane/carrying
    release_brake: Callable[[], Generator] # /crane/winch/brake/release
    engage_brake: Callable[[], Generator] # /crane/winch/brake/engage
    attach: Callable[[], Generator] # /crane/winch/attach
    detach: Callable[[], Generator] # /crane/winch/detach
    cell_pose: Callable[[str], Generator] # /cell_pose
    identify_pool: Callable[[str, float, float, float], Generator]
    
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

class MoveRod(StateMachine):

    source_cell: Const[str]
    target_cell: Const[str]

    source_cell_x: float = 0.0
    source_cell_y: float = 0.0
    target_cell_x: float = 0.0
    target_cell_y: float = 0.0
    cell_found: bool = False # last /cell_pose
    succeeded: bool = False # last service call
    carrying: bool = False # if the winch holds the rod
    cells: dict[str, str] = {} # map of every cell in the pool and its type 'fuel', 'control', 'empty'


    # robot platform
    send_target: Required[Callable[[float, float, float], None]]
    set_carrying: Required[Callable[[bool], None]]
    release_brake: Required[Callable[[], Generator]]
    engage_brake: Required[Callable[[], Generator]]
    attach: Required[Callable[[], Generator]]
    detach: Required[Callable[[], Generator]]
    cell_pose: Required[Callable[[str], Generator] ]
    identify_pool: Required[Callable[[str, float, float, float], Generator]]
    at_target: Required[Input[None]]

    # operation
    move_to: type[Operation] = MoveTo

    def initial(self):
        return self.ReleaseBrake

    class ReleaseBrake(State):

        def entry(self):
            self.succeeded = yield from self.release_brake()

        def transitions(self):
            if self.succeeded:
                return self.IdentifyPools
            self.logger.error('Brake release failed')
            return self.ReleaseBrake

    class IdentifyPools(State):

        def entry(self):
            for pool, centre_x in POOL_CENTERS.items():
                yield from self.move_to(centre_x, CANAL_Y, IDENTIFY_POOL_WINCH_Z)
                cells = yield from self.identify_pool(pool, centre_x, CANAL_Y, IDENTIFY_POOL_WINCH_Z)
                self.cells = {**self.cells, **cells}

        def transitions(self):
            cell_types = {cell_type: list(self.cells.values()).count(cell_type)
                      for cell_type in ('fuel', 'control', 'empty')}
            self.logger.info(f'Pool Cells: {cell_types}')
            return self.GetSource

    class GetSource(State):

        def entry(self):
            self.cell_found, self.source_cell_x, self.source_cell_y = yield from self.cell_pose(self.source_cell)

        def transitions(self):
            if self.cell_found:
                return self.GetTarget
            self.logger.error(f'Unknown cell {self.source_cell}')
            return FINAL

    class GetTarget(State):

        def entry(self):
            self.cell_found, self.target_cell_x, self.target_cell_y = yield from self.cell_pose(self.target_cell)

        def transitions(self):
            if self.cell_found:
                return self.CheckValidRodMove
            self.logger.error(f'Unknown cell {self.target_cell}')
            return FINAL

    class CheckValidRodMove(State):

        def transitions(self):
            if self.cells[self.source_cell] == 'empty':
                self.logger.error(f'{self.source_cell} is empty')
                return FINAL
            if self.cells[self.target_cell] != 'empty':
                self.logger.error(f'{self.target_cell} is not empty')
                return FINAL
            return self.MoveAboveSourceCell

    class MoveAboveSourceCell(State):

        def entry(self):
            x, y = apply_winch_wall_safety(self.source_cell_x, self.source_cell_y)
            yield from self.move_to(x, y, EMPTY_TRAVEL_WINCH_Z)

        def transitions(self):
            return self.LowerToCell

    class LowerToCell(State):

        def entry(self):
            if self.carrying:
                x, y = apply_winch_wall_safety(self.target_cell_x, self.target_cell_y)
            else:
                x, y = apply_winch_wall_safety(self.source_cell_x, self.source_cell_y)
            yield from self.move_to(x, y, LOWERED_WINCH_Z)

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
                    self.carrying = True
                    self.set_carrying(True)
                return self.RaiseFromCell, action
            self.logger.error('AttachRod failed')
            return self.AttachRod

    class RaiseFromCell(State):

        def entry(self):
            if self.carrying:
                yield from self.move_to(self.source_cell_x, self.source_cell_y, CARRY_WINCH_Z)
            else:
                yield from self.move_to(self.target_cell_x, self.target_cell_y, EMPTY_TRAVEL_WINCH_Z)

        def transitions(self):
            if self.carrying:
                return self.MoveToCanal
            self.logger.info(f'Moved rod from {self.source_cell} to {self.target_cell}')
            return FINAL

    class MoveToCanal(State):

        def entry(self):
            x, _ = apply_winch_wall_safety(self.source_cell_x, self.source_cell_y)
            yield from self.move_to(x, CANAL_Y, CARRY_WINCH_Z)

        def transitions(self):
            return self.MoveAlongCanal

    class MoveAlongCanal(State):

        def entry(self):
            x, _ = apply_winch_wall_safety(self.target_cell_x, self.target_cell_y)
            yield from self.move_to(x, CANAL_Y, CARRY_WINCH_Z)

        def transitions(self):
            return self.MoveAboveTargetCell

    class MoveAboveTargetCell(State):

        def entry(self):
            x, y = apply_winch_wall_safety(self.target_cell_x, self.target_cell_y)
            yield from self.move_to(x, y, CARRY_WINCH_Z)
            
        def transitions(self):
            return self.LowerToCell
            
    # LowerToCell

    class DetachRod(State):

        def entry(self):
            self.succeeded = yield from self.detach()

        def transitions(self):
            if self.succeeded:      
                def action():
                    self.carrying = False
                    self.set_carrying(False)
                    # update cell map
                    self.cells = {**self.cells, self.target_cell: self.cells[self.source_cell],
                                  self.source_cell: 'empty'}
                return self.RaiseFromCell, action
            self.logger.error('DetachRod failed')
            return self.DetachRod

    # RaiseFromCell -> FINAL