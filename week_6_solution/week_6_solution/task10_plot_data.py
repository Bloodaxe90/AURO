"""
Plot the barrel size against distance, as collected by task9_data_collector.

Usage: ros2 run week_6_solution task10_plot_data [csv_file]
where csv_file defaults to task9_data_collection.csv.
"""

import argparse
import csv
from typing import Optional

import matplotlib.pyplot as plt


def read_data(filename: str) -> tuple[list[float], list[float]]:
    """Return the size and distance columns of the CSV file."""
    sizes = []
    distances = []
    with open(filename, newline='') as csv_file:
        for row in csv.DictReader(csv_file):
            sizes.append(float(row['size']))
            distances.append(float(row['distance']))
    return sizes, distances


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description='Plot barrel size against distance.')
    parser.add_argument('csv_file', nargs='?', default='task9_data_collection.csv',
                        help='CSV file written by task9_data_collector '
                             '(default: %(default)s)')
    options = parser.parse_args(args)

    sizes, distances = read_data(options.csv_file)

    # Plot size vs. distance
    plt.figure()
    plt.scatter(sizes, distances)
    plt.xlabel('Size (bounding box area, pixels)')
    plt.ylabel('Distance (m)')
    plt.title('Size vs Distance')
    plt.show()


if __name__ == '__main__':
    main()
