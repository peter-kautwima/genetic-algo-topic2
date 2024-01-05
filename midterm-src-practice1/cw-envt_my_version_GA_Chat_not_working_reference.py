import pybullet as p
import pybullet_data
import time
import numpy as np
import random
from creature import Creature
import math
import matplotlib.pyplot as plt

class GeneticAlgorithm:
    def __init__(self, population_size, mutation_rate, crossover_rate, gene_count, tournament_size):
        self.population_size = population_size
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.gene_count = gene_count
        self.tournament_size = tournament_size
        self.population = self.initialize_population()
        self.best_fitnesses = []

    def initialize_population(self):
    # Initialize your population here
        return [Creature(gene_count=self.gene_count) for _ in range(self.population_size)]

    def run_generation(self):
        fitness_scores = [self.fitness_function(creature) for creature in self.population]
        self.best_fitnesses.append(max(fitness_scores))
        parents = self.selection(fitness_scores)
        offspring = self.crossover(parents)
        self.population = self.mutation(offspring)
        sorted_population = [x for _, x in sorted(zip(fitness_scores, self.population), reverse=True)]
        elites = sorted_population[:int(self.population_size * 0.1)]
         # Evaluate fitness
        fitness_scores = [self.fitness_function(creature) for creature in self.population]
        self.population = elites + offspring[:len(self.population) - len(elites)]

    def selection(self, fitness_scores):
        selected = []
        for _ in range(self.population_size):
            tournament = np.random.choice(self.population, size=self.tournament_size)
            tournament_fitnesses = [self.fitness_function(creature) for creature in tournament]
            winner = np.argmax(tournament_fitnesses)
            selected.append(tournament[winner])
        return selected

    def crossover(self, parents):
        return parents

    def mutation(self, offspring):
        mutated_offspring = []
        for creature in offspring:
            if np.random.random() < self.mutation_rate:
                # Apply mutation to creature here
                pass
            mutated_offspring.append(creature)
        return mutated_offspring

    def fitness_function(self, creature):
        return creature.get_distance_travelled()

ga = GeneticAlgorithm(population_size=100, mutation_rate=0.01, crossover_rate=0.7, gene_count=3, tournament_size=5)

for i in range(100):
    ga.run_generation()

plt.plot(ga.best_fitnesses)
plt.xlabel('Generation')
plt.ylabel('Best Fitness')
plt.show()


def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    # Function to create a mountain with rocks
    def gaussian(x, y, sigma=arena_size/4):
        return mountain_height * math.exp(-((x**2 + y**2) / (2 * sigma**2)))

    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = gaussian(x, y)
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)


def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
    # Function to create random rocks in the arena
    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = 0.5
        size = random.uniform(0.1,max_size)
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)


def make_arena(arena_size=10, wall_height=1):
    # Function to create the arena with walls and floor
    wall_thickness = 0.5
    floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness])
    floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1])
    floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2])


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

arena_size = 20
make_arena(arena_size=arena_size)

mountain_position = (10, 10, 1)
mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
p.setAdditionalSearchPath('shapes/')
mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)
mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position, mountain_orientation, useFixedBase=1)

p.setRealTimeSimulation(1)
