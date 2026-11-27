from typing import Callable, Generator
from week_4.robochart_fsm import FINAL, Required, Scope, State, StateMachine
from solution.task1.environment_helper import SEARCH_PLAN, door_pose, look_pose


NAV_TIMEOUT = 1200 # Max time allowed for one Nav2 goal (120s)

MAX_NAV_ATTEMPTS = 3 # Max Nav2 goals sent to the same place before giving up
MAX_TAG_IDENTIFICATION_ATTEMPTS = 3 # Max tag identification attempts in a row before the camera is assumed faulty


class RobotPlatform(Scope):
    navigate_to: Callable[[float, float, float, int], Generator]
    identify_tags: Callable[[], Generator]  # /camera/image_raw
    robot_pose: Callable[[], tuple]
    stop: Callable[[], None]


class FetchEndEffector(StateMachine):

    looks: list[tuple[str, str]] = [] # room wall pairs still to look at
    current_room: str = '' # room currently in

    arrived: bool = False # if last Nav2 goal succeeded
    tags_identified: bool = False # if the camera identified the tags

    attempts: int = 0

    # robot platform
    navigate_to: Required[Callable[[float, float, float, int], Generator]]
    identify_tags: Required[Callable[[], Generator]]
    robot_pose: Required[Callable[[], tuple]]
    stop: Required[Callable[[], None]]

    def initial(self):
        def action():
            self.looks = list(SEARCH_PLAN)
        return self.Search, action

    class Search(State):

        def entry(self):
            yield # so on_change logs state

        def transitions(self):
            if self.looks:
                def action():
                    self.attempts = 0
                return self.LookAtWall, action

            self.logger.error('Failed to find target end effector in any room, giving up')
            return self.Fault

    class LookAtWall(State):

        def entry(self):
            self.attempts = self.attempts + 1
            self.tags_identified = True
            room, wall = self.looks[0]

            # go through the door first if not in the room yet
            if room != self.current_room:
                x, y, yaw = door_pose(room)
                self.arrived = yield from self.navigate_to(x, y, yaw,
                                                           NAV_TIMEOUT)
                if not self.arrived:
                    return
                self.current_room = room

            x, y, yaw = look_pose(room, wall)
            self.arrived = yield from self.navigate_to(x, y, yaw, NAV_TIMEOUT)
            if not self.arrived:
                return

            tags = yield from self.identify_tags()
            if tags is None:
                self.tags_identified = False
                return  # no images from the camera

            self.logger.info(f'Tags seen on {wall} wall of {room} room: '
                             f'{sorted(tag_id for tag_id, _ in tags)}')

        def transitions(self):
            if not self.tags_identified:
                if self.attempts < MAX_TAG_IDENTIFICATION_ATTEMPTS:
                    self.logger.warning(
                        f'Failed to identify tags (try {self.attempts}), trying again')
                    return self.LookAtWall
                self.logger.error(
                    f'Failed to identify tags after {MAX_TAG_IDENTIFICATION_ATTEMPTS} attempts, giving up')
                return self.Fault

            if not self.arrived:
                if self.attempts < MAX_NAV_ATTEMPTS:
                    self.logger.warning(f'Failed to look at {self.looks[0][1]} wall of '
                                        f'{self.looks[0][0]} room (try {self.attempts}), trying again')
                    return self.LookAtWall

                # cannot get there so skip wall
                self.logger.warning(f'Failed to look at {self.looks[0][1]} wall of '
                                    f'{self.looks[0][0]} room after {MAX_NAV_ATTEMPTS} attempts, moving on')
                def action():
                    self.looks = self.looks[1:]
                    self.current_room = ''
                return self.Search, action

            def action():
                self.looks = self.looks[1:]
            return self.Search, action

    class Fault(State):

        def entry(self):
            # if catastrophic failure stop the robot so it doesnt keep moving
            self.stop()
            self.logger.error('Major fault, stopping the robot, human intervention needed')

        def transitions(self):
            return FINAL