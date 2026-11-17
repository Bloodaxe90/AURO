from typing import Callable, Generator
from week_4.robochart_fsm import Const, FINAL, Input, Operation, Param, Required, Scope, State, \
    StateMachine

CARRY_LIFT = -4.0 # when carrying a rod to clear all cells
LOWERED_LIFT = -6.38 # to touch the rod before attachment
CANAL_Y = 0.0 # the cannal bit between reator and storage

class CranePlatform(Scope):
    send_target: Callable[[float, float, float], None] # publish [x, y, z] on /crane/position_commands
    set_carrying: Callable[[bool], None] # publish on /crane/carrying
    release_brake: Callable[[], Generator] # /crane/winch/brake/release
    engage_brake: Callable[[], Generator] # /crane/winch/brake/engage
    attach: Callable[[], Generator] # /crane/winch/attach
    detach: Callable[[], Generator] # /crane/winch/detach
    cell_pose: Callable[[str], Generator] # /cell_pose
    
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

    # robot platform
    send_target: Required[Callable[[float, float, float], None]]
    set_carrying: Required[Callable[[bool], None]]
    release_brake: Required[Callable[[], Generator]]
    engage_brake: Required[Callable[[], Generator]]
    attach: Required[Callable[[], Generator]]
    detach: Required[Callable[[], Generator]]
    cell_pose: Required[Callable[[str], Generator] ]
    at_target: Required[Input[None]]

    # operation
    move_to: type[Operation] = MoveTo

    def initial(self):
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
                return self.ReleaseBrake
            self.logger.error(f'Unknown cell {self.target_cell}')
            return FINAL

    class ReleaseBrake(State):

        def entry(self):
            self.succeeded = yield from self.release_brake()

        def transitions(self):
            if self.succeeded:
                return self.MoveAboveSourceCell
            self.logger.error('Brake release failed')
            return self.ReleaseBrake

    class MoveAboveSourceCell(State):

        def entry(self):
            yield from self.move_to(self.source_cell_x, self.source_cell_y, CARRY_LIFT)

        def transitions(self):
            return self.LowerToCell

    class LowerToCell(State):

        def entry(self):
            if self.carrying:
                yield from self.move_to(self.target_cell_x, self.target_cell_y, LOWERED_LIFT)
            else:
                yield from self.move_to(self.source_cell_x, self.source_cell_y, LOWERED_LIFT)

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
                yield from self.move_to(self.source_cell_x, self.source_cell_y, CARRY_LIFT)
            else:
                yield from self.move_to(self.target_cell_x, self.target_cell_y, CARRY_LIFT)

        def transitions(self):
            if self.carrying:
                return self.MoveToCanal
            self.logger.info(f'Moved rod from {self.source_cell} to {self.target_cell}')
            return FINAL

    class MoveToCanal(State):

        def entry(self):
            yield from self.move_to(self.source_cell_x, CANAL_Y, CARRY_LIFT)

        def transitions(self):
            return self.MoveAlongCanal

    class MoveAlongCanal(State):

        def entry(self):
            yield from self.move_to(self.target_cell_x, CANAL_Y, CARRY_LIFT)

        def transitions(self):
            return self.MoveAboveTargetCell

    class MoveAboveTargetCell(State):

        def entry(self):
            yield from self.move_to(self.target_cell_x, self.target_cell_y, CARRY_LIFT)

        def transitions(self):
            return self.LowerToCell
            
    # LowerToCell -> DetachRod

    class DetachRod(State):

        def entry(self):
            self.succeeded = yield from self.detach()

        def transitions(self):
            if self.succeeded:      
                def action():
                    self.carrying = False
                    self.set_carrying(False)
                return self.RaiseFromCell, action
            self.logger.error('DetachRod failed')
            return self.DetachRod

    # RaiseFromCell -> FINAL