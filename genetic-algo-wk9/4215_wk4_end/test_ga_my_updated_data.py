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

class TestGA(unittest.TestCase):
    def testBasicGA(self):
        pop = population.Population(pop_size=10, gene_count=3)
        sim = simulation.Simulation()

        # Step 1: Store data in a list of dictionaries
        data = []

        for iteration in range(1000):
            for cr in pop.creatures:
                sim.run_creature(cr, 2400)            
            fits = [cr.get_distance_travelled() for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            # Store the data instead of printing
            data.append({
                'iteration': iteration,
                'fittest': np.round(np.max(fits), 3),
                'mean': np.round(np.mean(fits), 3),
                'mean_links': np.round(np.mean(links)),
                'max_links': np.round(np.max(links))
            })

            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
            for i in range(len(pop.creatures)):
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=0.25)
                dna = genome.Genome.grow_mutate(dna, rate=0.1)
                cr = creature.Creature(1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_distance_travelled() == max_fit:
                    new_cr = creature.Creature(1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = "elite_"+str(iteration)+".csv"
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            pop.creatures = new_creatures

        # Step 2: Convert list of dictionaries to DataFrame
        df = pd.DataFrame(data)

        # Step 3: Save DataFrame to CSV
        df.to_csv('iteration_data.csv', index=False)
                            
        self.assertNotEqual(fits[0], 0)

unittest.main()