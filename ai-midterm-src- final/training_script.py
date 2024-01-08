import os
import glob
import subprocess

def main():
    # Run the genetic algorithm
    subprocess.run(["python", "test_ga_no_threads_keeps_value_elites.py"], check=True)

    for csv_file in glob.glob('*.csv'):
        if csv_file == 'ga_output.csv':
            continue  # Ignore ga_output.csv file
        subprocess.run(["python", "realtime_from_csv_my_version.py", csv_file], check=True)

if __name__ == "__main__":
    print(main())