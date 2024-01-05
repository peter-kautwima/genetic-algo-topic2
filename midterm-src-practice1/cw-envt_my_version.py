import pybullet as p
import pybullet_data
import time
import numpy as np
import random
from creature import Creature


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

import random
import pybullet as p
import math

import pybullet as p
import numpy as np
import matplotlib.pyplot as plt

class GeneticAlgorithm:
    def __init__(self, population_size, mutation_rate, crossover_rate):
        self.population_size = population_size
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.population = self.initialize_population()
        self.best_fitnesses = []

    def initialize_population(self):
        # Initialize your population here
        return [Creature() for _ in range(self.population_size)]

    def run_generation(self):
        # Evaluate fitness
        fitness_scores = [self.fitness_function(creature) for creature in self.population]

        # Keep track of the best fitness
        self.best_fitnesses.append(max(fitness_scores))

        # Select parents
        parents = self.selection(fitness_scores)
        # Perform crossover
        offspring = self.crossover(parents)
        # Perform mutation
        self.population = self.mutation(offspring)

        # Implement elitism: select the best individuals from the current population
        sorted_population = [x for _, x in sorted(zip(fitness_scores, self.population), reverse=True)]
        elites = sorted_population[:int(self.population_size * 0.1)]  # top 10%

        # Create new generation: combine elites and offspring
        self.population = elites + mutated_offspring[:len(self.population) - len(elites)]
    
        def selection(self, fitness_scores):
        # Implement a selection method here, such as roulette wheel selection or tournament selection
        # This is just a placeholder. Replace this with your actual code.
        return np.random.choice(self.population, size=self.population_size, p=fitness_scores)

        def selection(self, fitness_scores):
            # Implement tournament selection
            selected = []
            for _ in range(self.population_size):
                # Select individuals for the tournament
                tournament = np.random.choice(self.population, size=self.tournament_size)
                # Evaluate fitness for the tournament individuals
                tournament_fitnesses = [self.fitness_function(creature) for creature in tournament]
                # Select the best individual among the tournament
                winner = np.argmax(tournament_fitnesses)
                selected.append(tournament[winner])
            return selected

    def crossover(self, parents):
        # Implement a crossover method here, such as single-point crossover or uniform crossover
        # This is just a placeholder. Replace this with your actual code.
        return parents

    def mutation(self, offspring):
        # Implement a mutation method here
        # This is just a placeholder. Replace this with your actual code.
        for creature in offspring:
            if np.random.random() < self.mutation_rate:
                creature.mutate()
        return offspring

    def fitness_function(self, creature):
        # Calculate the fitness based on the creature's ability to climb the mountain
        # This is just a placeholder. Replace this with your actual code.
        return creature.climb_mountain()

# Initialize the genetic algorithm
ga = GeneticAlgorithm(population_size=100, mutation_rate=0.01, crossover_rate=0.7)

# Run the genetic algorithm for a certain number of generations
for i in range(100):
    ga.run_generation()

# Plot the best fitness over time
plt.plot(ga.best_fitnesses)
plt.xlabel('Generation')
plt.ylabel('Best Fitness')
plt.show()


def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    def gaussian(x, y, sigma=arena_size/4):
        """Return the height of the mountain at position (x, y) using a Gaussian function."""
        return mountain_height * math.exp(-((x**2 + y**2) / (2 * sigma**2)))

    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = gaussian(x, y)  # Height determined by the Gaussian function

        # Adjust the size of the rocks based on height. Higher rocks (closer to the peak) will be smaller.
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor

        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)



def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = 0.5  # Adjust based on your needs
        size = random.uniform(0.1,max_size)
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)


def make_arena(arena_size=10, wall_height=1):
    wall_thickness = 0.5
    floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness])
    floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1])
    floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    # Create four walls
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2])


p.setGravity(0, 0, -10)

arena_size = 20
make_arena(arena_size=arena_size)

#make_rocks(arena_size=arena_size)

mountain_position = (10, 10, 1)  # Adjust as needed
mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
p.setAdditionalSearchPath('shapes/')
# mountain = p.loadURDF("mountain.urdf", mountain_position, mountain_orientation, useFixedBase=1)
# mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position, mountain_orientation, useFixedBase=1)

mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)

# works also load in the other ones now! see the prepareshapes
mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position, mountain_orientation, useFixedBase=1)

# Load the landscape into your PyBullet environment
landscape = p.loadURDF("mountain.urdf", useFixedBase=True)
# generate a random creature
cr = creature.Creature(gene_count=3)
# save it to XML
with open('test.urdf', 'w') as f:
    f.write(cr.to_xml())
# load it into the sim
rob1 = p.loadURDF('test.urdf', (0, 0, 10))


p.setRealTimeSimulation(1)

