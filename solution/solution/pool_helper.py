POOL_CENTERS = {'reactor': -5.75, 'storage': 5.75}
POOL_X = {'reactor': (-9.0, -2.5), 'storage': (2.5, 9.0)} # inside faces of the pool walls
POOL_Y = (-2.25, 2.25)
CANAL_Y = 0.0 # centre line of the canal between the pools

CRANE_WIDTH = 0.5
WALL_CRANE_SAFETY_MARGIN = 0.01 # always keep this far from pool walls and dividers when close to pool rim
                                # Must be less than 0.025 since the cells give the rods this much play (0.4 - 0.35) / 2 = 0.025


def pool_of(x):
    # the reactor negative x storage at positive x
    return 'reactor' if x < 0.0 else 'storage'

def canal_end_x(x):
    # end x of the pool that opens onto the canal
    return POOL_X['reactor'][1] if pool_of(x) == 'reactor' else POOL_X['storage'][0]

def apply_crane_wall_safety(x, y, waypoint_tolerance=0.0):
    # Keeps the crane inside the pool walls. for waypoint its kept much further from the walls
    bottom_x, top_x = POOL_X[pool_of(x)]
    safety_margin = CRANE_WIDTH / 2.0 + WALL_CRANE_SAFETY_MARGIN + waypoint_tolerance
    return (min(max(x, bottom_x + safety_margin), top_x - safety_margin),
            min(max(y, POOL_Y[0] + safety_margin), POOL_Y[1] - safety_margin))

def route(from_x, to_x, to_y, waypoint_tolerance):
    # Waypoints for the winch: straight there if the route is in the same pool else through the canal
    if pool_of(from_x) == pool_of(to_x):
        return [(to_x, to_y)]
    from_mouth_x, _ = apply_crane_wall_safety(canal_end_x(from_x), CANAL_Y, waypoint_tolerance)
    to_mouth_x, _ = apply_crane_wall_safety(canal_end_x(to_x), CANAL_Y, waypoint_tolerance)
    return [(from_mouth_x, CANAL_Y), (to_mouth_x, CANAL_Y), (to_x, to_y)]
