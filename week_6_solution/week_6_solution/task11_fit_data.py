"""
Fit a power law, distance = a * size ** c, to the data collected by task9_data_collector.

Usage: ros2 run week_6_solution task11_fit_data [csv_file] [options]
where csv_file defaults to task9_data_collection.csv; see --help for the options.

The fit is a weighted nonlinear least-squares fit using scipy's curve_fit. The plot of the data
and fitted curve is saved as a PNG.
"""

import argparse
import csv
from typing import Optional

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

matplotlib.use('Agg')  # We only save the figure, so no display is needed.

# We could have used pandas instead to load the data.
def read_data(filename: str) -> tuple[np.ndarray, np.ndarray]:
    """Return the size and distance columns of the CSV file, keeping only positive values."""
    sizes = []
    distances = []
    with open(filename, newline='') as csv_file:
        for row in csv.DictReader(csv_file):
            size, distance = float(row['size']), float(row['distance'])
            if size > 0 and distance > 0:
                sizes.append(size)
                distances.append(distance)
    return np.array(sizes), np.array(distances)

# The equation to be fitted.
def model_f(x: np.ndarray, a: float, c: float) -> np.ndarray:
    """Power-law model."""
    return a * (x ** c)

# Fitting using curve_fit.
def fit_power_law(x: np.ndarray, y: np.ndarray, weights: np.ndarray) -> tuple[float, float]:
    """Return (a, c) of the power law best fitting the data."""
    # curve_fit uses sigma = std deviation -> smaller sigma = more weight
    sigma = 1 / weights
    param, _ = curve_fit(model_f, x, y, sigma=sigma, absolute_sigma=False, maxfev=10000)
    return float(param[0]), float(param[1])

# Main program with parser arguments
def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description='Fit a power law to barrel size and distance.')
    parser.add_argument('csv_file', nargs='?', default='task9_data_collection.csv',
                        help='CSV file written by task9_data_collector (default: %(default)s)')
    parser.add_argument('--start-row', type=int, default=100,
                        help='first row to use, to skip outliers (default: %(default)s)')
    parser.add_argument('--end-row', type=int, default=None,
                        help='row to stop before, to skip outliers (default: use all)')
    parser.add_argument('--weighted-rows', type=int, default=100,
                        help='number of final rows given more weight (default: %(default)s)')
    parser.add_argument('--weight', type=float, default=100.0,
                        help='weight of those final rows (default: %(default)s)')
    parser.add_argument('--output', default='fitted-graph.png',
                        help='PNG file for the plot (default: %(default)s)')
    options = parser.parse_args(args)

    sizes, distances = read_data(options.csv_file)

    # Process from start_row to end_row (to remove some outliers).
    x_data = sizes[options.start_row:options.end_row]
    y_data = distances[options.start_row:options.end_row]
    if len(x_data) < 2:
        parser.error(f'only {len(x_data)} usable rows in {options.csv_file} '
                     f'after --start-row/--end-row; need at least 2')

    # Give the final rows more weight, since they are expected to be closer to the barrel,
    # therefore optimising fitting towards closer distances.
    weights = np.ones(len(x_data))
    if options.weighted_rows > 0:
        weights[-options.weighted_rows:] = options.weight

    # --- Fit power-law ---
    a_power, c_power = fit_power_law(x_data, y_data, weights)
    print(f'Power-law fit: a={a_power:.3f}, c={c_power:.3f}')

    # --- Generate model curve for plotting ---
    x_model = np.linspace(x_data.min(), x_data.max(), 100)
    y_power_model = model_f(x_model, a_power, c_power)

    # --- Plot ---
    plt.figure(figsize=(8, 6))
    plt.scatter(x_data, y_data, facecolors='none', edgecolors='r', label='data')
    plt.plot(x_model, y_power_model, color='g', label='power-law fit')
    plt.xlabel('Size (bounding box area, pixels)')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(options.output)
    print(f'Plot saved to {options.output}')


if __name__ == '__main__':
    main()
