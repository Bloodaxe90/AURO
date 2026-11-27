import math

ROBOT_START = (0.0, -3.5)

DROPOFF_CENTER = (10.85, 0.0) # centre of drop off platform
DROPOFF_YAW = 0.0 # make robot face east so end effector is dropped behind it

# storage room centers
ROOM_CENTERS = {'blue': (-5.683, 10.0), 'orange': (5.683, 10.0),
         'yellow': (-5.683, -10.0), 'purple': (5.683, -10.0)}
ROOM_LENGTH = 4.6 # y length of a room from the front to the back walls

WALL_NORMALS = {'west': 1.0, 'east': -1.0} # x direction pointing from the wall into the room (the x normals from the wall)
TAG_WALL_X = 2.294 # x distance from the rooms center to where the tags lie on the west and east walls

# order to search the rooms and their walls
SEARCH_PLAN = [('purple', 'west'), ('purple', 'east'),
               ('orange', 'east'), ('orange', 'west'),
               ('blue', 'east'), ('blue', 'west'),
               ('yellow', 'west'), ('yellow', 'east')]

LOOK_DISTANCE = 3.3 # x distance to keep away from wall so tags stay in camera FOV
DOOR_OFFSET = 0.8 # y distance from the rooms doors

EFFECTOR_TO_TAG_DISTANCE = 0.48 # left distance from tag on wall to end effector
EFFECTOR_OUT_FROM_WALL_DISTANCE = 0.3 # distance the end effector is from the wall
EFFECTOR_APPROACH_DISTANCE = 0.75 # distance to maintain away from end effectors origin


def world_to_map(x, y):
    return x - ROBOT_START[0], y - ROBOT_START[1]

def map_to_world(x, y):
    return x + ROBOT_START[0], y + ROBOT_START[1]

def facing(x, y):
    # the yaw given a direction (x,y)
    return math.atan2(y, x)

def door_pose(room):
    # position to stand outside door or room
    centre_x, centre_y = ROOM_CENTERS[room]
    into_room = 1.0 if centre_y > 0 else -1.0
    return centre_x, centre_y - into_room * (ROOM_LENGTH / 2.0 + DOOR_OFFSET), facing(0.0, into_room)

def wall_x(room, wall):
    # x position of wall
    centre_x, _ = ROOM_CENTERS[room]
    return centre_x - WALL_NORMALS[wall] * TAG_WALL_X

def look_pose(room, wall):
    # position to stand in room to look at tags
    _, centre_y = ROOM_CENTERS[room]
    normal_x = WALL_NORMALS[wall]
    return wall_x(room, wall) + normal_x * LOOK_DISTANCE, centre_y, facing(-normal_x, 0.0)

def tag_on_wall(room, wall, camera_x, camera_y, bearing):
    # position of tag on wall
    x = wall_x(room, wall)
    if abs(math.cos(bearing)) < 1e-3:
        return None
    distance = (x - camera_x) / math.cos(bearing)
    if distance <= 0.0:
        return None
    y = camera_y + distance * math.sin(bearing)
    _, centre_y = ROOM_CENTERS[room]
    if abs(y - centre_y) > ROOM_LENGTH / 2.0:
        return None
    return x, y

def effector_from_tag(wall, tag_x, tag_y):
    # position of end effector from the tag
    normal_x = WALL_NORMALS[wall]
    return tag_x + normal_x * EFFECTOR_OUT_FROM_WALL_DISTANCE, tag_y - normal_x * EFFECTOR_TO_TAG_DISTANCE

def approach_pose(wall, effector_x, effector_y):
    # position to keep away from end effector when approaching it for pickup
    normal_x = WALL_NORMALS[wall]
    return effector_x + normal_x * EFFECTOR_APPROACH_DISTANCE, effector_y, facing(-normal_x, 0.0)