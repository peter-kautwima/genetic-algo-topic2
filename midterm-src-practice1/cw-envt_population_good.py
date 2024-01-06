import os 
import genome
import sys
import creature
import pybullet as p
import time 
import random
import numpy as np
from population import Population
from simulation import Simulation


"""
The code you provided is a genetic algorithm simulation for evolving creatures, while the code in realtime_from_csv.py is a real-time simulation of a single creature's movement. Here are the key differences:

1.Purpose: The provided code is a genetic algorithm that evolves a population of creatures over multiple generations. It simulates each creature, calculates their fitness, and then creates a new generation of creatures based on the fitness of the current generation. The realtime_from_csv.py script, on the other hand, simulates a single creature's movement in real-time.

2. Creature Creation: In the provided code, creatures are created and evolved using a genetic algorithm. The DNA of each creature is represented as a Genome object, which is mutated and crossed over to create new creatures. In realtime_from_csv.py, the creature's DNA is loaded from a CSV file, and the creature is created based on this DNA.

3. Simulation: The provided code uses a Simulation object to simulate each creature. The realtime_from_csv.py script directly uses the PyBullet API to simulate the creature's movement.

4. Fitness Calculation: The provided code calculates the fitness of each creature after simulating it, and uses this fitness to select parents for the next generation. The realtime_from_csv.py script does not calculate fitness; instead, it prints the total distance moved by the creature.

5. Population Management: The provided code manages a population of creatures using a Population object. It implements a genetic algorithm to evolve this population over multiple generations. The realtime_from_csv.py script does not manage a population; it only simulates a single creature.

In summary, the provided code is a genetic algorithm for evolving creatures, while the realtime_from_csv.py script is a real-time simulation of a single creature's movement.
"""

def main():
    p.connect(p.GUI)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    plane_shape = p.createCollisionShape(p.GEOM_PLANE)
    floor = p.createMultiBody(plane_shape, plane_shape)
    p.setGravity(0, 0, -10)

    # Create a population of creatures
    pop = Population(size=10, gene_count=3)
    # Create a simulation object
    sim = Simulation()

    # Run the genetic algorithm for a certain number of iterations
    for i in range(5):
        # Simulate each creature in the population
        for j in range(pop.size):
            cr = pop.creatures[j]
            dna = cr.get_dna()
            # Save the creature's DNA to a CSV file
            dna.to_csv('dna.csv')
            # Update the creature's DNA from the CSV file
            cr.update_dna(genome.Genome.from_csv('dna.csv'))
            # Save the creature to a URDF file
            with open('creature.urdf', 'w') as f:
                f.write(cr.to_xml())
            # Load the creature into the simulation
            rob = p.loadURDF('creature.urdf')
            # Air drop the creature
            p.resetBasePositionAndOrientation(rob, [0, 0, 2.5], [0, 0, 0, 1])
            # Run the creature in the simulation
            sim.run_creature(cr)
        # Calculate the fitness of each creature
        fitnesses = [cr.get_fitness() for cr in pop.creatures]
        # Create a fitness map
        fitness_map = pop.get_fitness_map(fitnesses)
        # Select parents and create new creatures
        new_creatures = []
        for j in range(pop.size):
            parent1 = pop.select_parent(fitness_map)
            parent2 = pop.select_parent(fitness_map)
            dna1 = pop.creatures[parent1].get_dna()
            dna2 = pop.creatures[parent2].get_dna()
            new_dna = dna1.crossover(dna2)
            new_dna.mutate()
            new_creature = creature.Creature(dna=new_dna)
            new_creatures.append(new_creature)
        # Implement elitism
        best_creature = pop.creatures[np.argmax(fitnesses)]
        new_creatures[0] = creature.Creature(dna=best_creature.get_dna())
        # Replace the old population with the new creatures
        pop.creatures = new_creatures

if __name__ == "__main__":
    main()
