import math

ROBOT_START = (0.0, -3.5)

def world_to_map(x, y):
    return x - ROBOT_START[0], y - ROBOT_START[1]

def map_to_world(x, y):
    return x + ROBOT_START[0], y + ROBOT_START[1]