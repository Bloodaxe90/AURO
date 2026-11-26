from solution.cell_identifier import cell_centres
from solution.pool_helper import pool_of

# Every cells centre (x, y)
CELL_XY = {name: (x, y) for pool in ('reactor', 'storage') for name, x, y in cell_centres(pool)}

MAX_POWER = 100.0 # %

MOVE_COST = 0.5 # How much one rod move costs

MIN_TRIM = 0.2 # if a trim causes the power to drop less than this % then stop trimming

# Reactor fuel rods to read the level of at a time and in all
READ_BATCH = 6

def in_reactor(cell):
    return pool_of(CELL_XY[cell][0]) == 'reactor'

def nearest_canal(cells):
    # cell closest to the canal (x = 0, y = 0) to ensure shortest moves
    return min(cells, key=lambda cell: abs(CELL_XY[cell][0]) + abs(CELL_XY[cell][1]))


def power_level(total_reactor_fuel, num_removed_control_rods, control_rod_coefficient):
    # same as reactor_monitor.power_level() (total reactor fuel * (1 + control_rod_coefficient x control rods not in the reactor))
    return total_reactor_fuel * (1.0 + control_rod_coefficient * num_removed_control_rods)


def best_move_combination(total_reactor_fuel, current_num_removed_control_rods, controls_left, storages_left, addable_storage_rod_power, control_rod_coefficient, target_power):
    # Tries every combination of control rods to remove and storage rods to insert keeping the
    # one that reaches the target power with the least waste (each move is counted as waste)
    # If none reach the target power without going over 100% it will keep the one that gets closest below 100%
    cheapest = None # cheapest combination of control and storage rods to move that reaches the target power
    closest = None # closest combination of control and storage rods to move that is below the target power
    for num_removed_control_rods in range(controls_left + 1): # + 1 to include removing all controlrods
        for num_added_storage_rods in range(storages_left + 1): # + 1 to include adding all storagerods
            predicted_power = power_level(total_reactor_fuel + num_added_storage_rods * addable_storage_rod_power,
                                current_num_removed_control_rods + num_removed_control_rods, control_rod_coefficient) # predicted power after moving the rods
            if predicted_power > MAX_POWER:
                continue
            total_rods_moved = num_removed_control_rods + num_added_storage_rods
            if predicted_power >= target_power:
                cost = (predicted_power - target_power) + MOVE_COST * total_rods_moved
                if cheapest is None or cost < cheapest[0]:
                    cheapest = (cost, num_removed_control_rods, num_added_storage_rods)
            elif closest is None or (predicted_power, -total_rods_moved) > closest[0]:
                closest = ((predicted_power, -total_rods_moved), num_removed_control_rods, num_added_storage_rods)
    choice = cheapest or closest
    if choice is None:
        return 0, 0
    return choice[1], choice[2]



def next_rod_move(cells, current_power, target_power, control_rod_coefficient,
                                 full_fuel_rod_power, storage_rod_level, rod_levels):
    '''
    Chooses the next rod to move as (source cell, target cell, reason) or None

    cells: every cell and what is in it 'fuel', 'control', 'empty'
    current_power: the current power level (% of maximum)
    target_power: the target power level (% of maximum)
    control_rod_coefficient: power multiplier per control rod removed
    full_fuel_rod_power: % of maximum power for one fuel rod at level 100
    storage_rod_level: level of the storage rods in storage pool
    '''
    reactor_control_rods = [c for c, rod in cells.items() if in_reactor(c) and rod == 'control']
    reactor_fuel_rods = [c for c, rod in cells.items() if in_reactor(c) and rod == 'fuel']
    storage_fuel_rods = [c for c, rod in cells.items() if not in_reactor(c) and rod == 'fuel']
    num_removed_control_rods = sum(1 for c, rod in cells.items() if not in_reactor(c) and rod == 'control')
    reactor_empty_cells = [c for c, rod in cells.items()
                     if in_reactor(c) and rod == 'empty' ]
    storage_empty_cells = [c for c, rod in cells.items()
                     if not in_reactor(c) and rod == 'empty']

    fuel_multiplier = 1.0 + control_rod_coefficient * num_removed_control_rods
    total_reactor_fuel = current_power / fuel_multiplier # sum of the inserted fuel rod power
    addable_storage_rod_power = full_fuel_rod_power * storage_rod_level / 100.0

    # If at or above the target power trim the waste by taking out the reactor fuel rods of those whose
    # level is known then take away the most power without going below the target
    if current_power >= target_power:
        if not storage_empty_cells:
            return None
        if current_power > MAX_POWER and reactor_fuel_rods:
            return nearest_canal(reactor_fuel_rods), nearest_canal(storage_empty_cells), 'over 100%'
        waste = current_power - target_power
        trims = []
        for cell in reactor_fuel_rods:
            if cell in rod_levels:
                drop = full_fuel_rod_power * rod_levels[cell] / 100.0 * fuel_multiplier
                if MIN_TRIM < drop <= waste:
                    trims.append((drop, cell))
        if not trims:
            return None
        drop, cell = max(trims)
        return (cell, nearest_canal(storage_empty_cells),
                f'Trimming waste: level {rod_levels[cell]:.2f} rod takes away {drop:.2f}%')

    num_removable_control_rods, num_addable_storage_rods = best_move_combination(
        total_reactor_fuel, num_removed_control_rods,
        min(len(reactor_control_rods), len(storage_empty_cells)), # cant remove more control rods than there are in the reactor
        min(len(storage_fuel_rods), len(reactor_empty_cells)), # cant insert more storage rods than there are empty cells in the reactor
        addable_storage_rod_power, control_rod_coefficient, target_power)
    if num_removable_control_rods == 0 and num_addable_storage_rods == 0:
        return None # nothing left that adds power

    # do the move that adds the most power first
    control_rod_gain = total_reactor_fuel * control_rod_coefficient
    storage_rod_gain = addable_storage_rod_power * fuel_multiplier
    if num_removable_control_rods > 0 and (num_addable_storage_rods == 0 or control_rod_gain >= storage_rod_gain):
        return (nearest_canal(reactor_control_rods), nearest_canal(storage_empty_cells),
                f'Removing control rod: planned {num_removable_control_rods} control rod, {num_addable_storage_rods} storage rod moves')
    return (nearest_canal(storage_fuel_rods), nearest_canal(reactor_empty_cells),
            f'Adding storage rod: planned {num_removable_control_rods} control rod, {num_addable_storage_rods} storage rod moves')



def rods_to_read(cells, rod_levels, read_cells, current_power, target_power, crane_x, crane_y):
    # next reactor fuel rods to read the level of (nearest first)
    if current_power - target_power <= MIN_TRIM:
        return []
    unknown = [c for c, rod in cells.items() if in_reactor(c) and rod == 'fuel'
               and c not in rod_levels and c not in read_cells]
    # nearest neighbour order
    reads = []
    x, y = crane_x, crane_y
    while unknown and len(reads) < READ_BATCH:
        cell = min(unknown, key=lambda c: abs(CELL_XY[c][0] - x) + abs(CELL_XY[c][1] - y))
        reads.append(cell)
        unknown.remove(cell)
        x, y = CELL_XY[cell]
    return reads


def new_target_cell(cells, target_cell):
    # An empty cell in the same pool as target_cell
    empty = [cell for cell, rod in cells.items()
             if rod == 'empty' and in_reactor(cell) == in_reactor(target_cell)]
    return nearest_canal(empty) if empty else None
