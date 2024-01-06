import os
import glob
import test_ga_no_threads
import offline_csv_my_version
import realtime_from_csv

def main(ga_generations, ga_output_path, best_dna_path):
    # Run the genetic algorithm
    test_ga_no_threads.run(ga_generations, ga_output_path)

    # Select the best DNA
    for csv_file in glob.glob('*.csv'):
        offline_csv_my_version.run(csv_file)

    # Simulate the best creature
    realtime_from_csv.run(best_dna_path)

if __name__ == "__main__":
    main(5, 'ga_output.csv', 'best_dna.csv')