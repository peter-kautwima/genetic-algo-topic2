import os
import glob
import subprocess

def main(ga_output_path, best_dna_path):
    # Run the genetic algorithm
    subprocess.run(["python", "test_ga_no_threads.py"], check=True)

    # Select the best DNA
    for csv_file in glob.glob('*.csv'):
        subprocess.run(["python", "offline_csv_my_version.py", csv_file], check=True)

    # Simulate the best creature
    subprocess.run(["python", "realtime_from_csv.py", best_dna_path], check=True)

if __name__ == "__main__":
    main('ga_output.csv', 'best_dna.csv')