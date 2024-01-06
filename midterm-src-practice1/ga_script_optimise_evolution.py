import os
import glob
import subprocess
from offline_csv_my_version import calculate_distances, find_best_csv


def main():
    # Run the genetic algorithm
    subprocess.run(["python", "test_ga_no_threads.py"], check=True)

    # Process all CSV files with offline_csv_my_version.py
    for csv_file in glob.glob('*.csv'):
        subprocess.run(["python", "offline_csv_my_version.py", csv_file], check=True)

    # Calculate the distances and find the best CSV file
    distances = calculate_distances()
    best_csv_file = find_best_csv(distances)

    # Simulate the best creature
    subprocess.run(["python", "realtime_from_csv_copy.py", best_csv_file], check=True)
    return best_csv_file

if __name__ == "__main__":
    print(main())