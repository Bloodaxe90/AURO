"""
Plot step responses from gain_tuner.py

python3 plot_steps.py target csv_file1 csv_file2

Saves the graph as steps.png
"""

import argparse
import csv
from typing import Optional
from solution.tuning.gain_tuner import TOLERANCE
import matplotlib
matplotlib.use('Agg')  # works withought display
import matplotlib.pyplot as plt

def read_data(filename: str) -> tuple[list[float], list[float]]:
    """Return the time and position columns of the CSV file."""
    times = []
    positions = []
    with open(filename, newline='') as csv_file:
        for row in csv.DictReader(csv_file):
            times.append(float(row['time']))
            positions.append(float(row['position']))
    return times, positions


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description='Plot step responses from gain_tuner.py.')
    parser.add_argument('target', type=float, help='target position of the step (m)')
    parser.add_argument('csv_files', nargs='+', help='CSV files written by gain_tuner.py')
    options = parser.parse_args(args)

    plt.figure()
    for filename in options.csv_files:
        times, positions = read_data(filename)
        plt.plot(times, positions, label=filename.removesuffix('.csv'))
    # target
    plt.axhline(options.target, color='grey', linestyle='--', label='target')
    plt.axhspan(options.target - TOLERANCE, options.target + TOLERANCE, color='grey', alpha=0.2)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Step response')
    plt.legend()
    plt.savefig('steps.png')


if __name__ == '__main__':
    main()