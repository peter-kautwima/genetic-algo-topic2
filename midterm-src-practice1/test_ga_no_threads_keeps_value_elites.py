# If you on a Windows machine with any Python version 
# or an M1 mac with any Python version
# or an Intel Mac with Python > 3.7
# the multi-threaded version does not work
# so instead, you can use this version. 

import unittest
import population
import simulation 
import genome 
import creature 
import numpy as np
import pandas as pd

import sys

# Set a default value for ga_generations
ga_generations = 50

# Check if a value was passed as a command-line argument
if len(sys.argv) > 1:
    try:
        # Try to convert the first command-line argument to an integer
        ga_generations = int(sys.argv[1])
    except ValueError:
        # If the conversion fails, print an error message and exit
        print(f"Error: Expected an integer, got {sys.argv[1]}")
        sys.exit(1)


class TestGA(unittest.TestCase):
    def testBasicGA(self):
        pop = population.Population(pop_size=10, 
                                    gene_count=3)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation()

        # Initialize an empty list to store the data for each generation
        data = []

        for iteration in range(ga_generations):
            # Initialize an empty list to store the total vertical distances travelled
            total_vertical_distances = []
            # this is a non-threaded version 
            # where we just call run_creature instead
            # of eval_population
            for cr in pop.creatures:
                sim.run_creature(cr, 2400)     
                # Append the total vertical distance travelled by the current creature
                total_vertical_distances.append(cr.get_total_vertical_distance_travelled())       
            #sim.eval_population(pop, 2400)
            fits = [cr.get_total_vertical_distance_travelled() for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) 
                    for cr in pop.creatures]
            print(iteration, "fittest:", np.round(np.max(fits), 3), 
                  "mean:", np.round(np.mean(fits), 3), "mean links", np.round(np.mean(links)), "max links", np.round(np.max(links)))       
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
            for i in range(len(pop.creatures)):
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]
                # now we have the parents!
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=0.25)
                dna = genome.Genome.grow_mutate(dna, rate=0.1)
                cr = creature.Creature(1)
                cr.update_dna(dna)
                new_creatures.append(cr)
            # elitism
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_total_vertical_distance_travelled() == max_fit:
                    new_cr = creature.Creature(1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = "elite_"+str(iteration)+".csv"
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

            # Store the data for this generation in a dictionary
            generation_data = {
                "iteration": iteration,
                "fittest": np.round(np.max(fits), 3),
                "mean": np.round(np.mean(fits), 3),
                "mean links": np.round(np.mean(links)),
                "max links": np.round(np.max(links)),
                "max vertical distance": np.round(np.max(total_vertical_distances), 3),
                "mean vertical distance": np.round(np.mean(total_vertical_distances), 3),
            }
    
            # Add the dictionary to the list
            data.append(generation_data)

        # Convert the list of dictionaries to a DataFrame
        df = pd.DataFrame(data)

        # Save the DataFrame to a CSV file
        df.to_csv("ga_output.csv", index=False)
                            
        self.assertNotEqual(fits[0], 0)

unittest.main()
