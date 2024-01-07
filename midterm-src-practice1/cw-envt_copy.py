import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import creature
import genome
import os 
import sys
import math
import population
import simulation 
from population import Population
from genome import Genome
from creature import Creature
import pandas as pd
import matplotlib.pyplot as plt

def main():
    # assert os.path.exists(csv_file), "Tried to load " + csv_file + " but it does not exists"

    p.connect(p.GUI)
    # not sure if I need th eone below. 
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

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

    mountain_position = (0, 0, -1)  # Adjust as needed
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    p.setAdditionalSearchPath('shapes/')
    # mountain = p.loadURDF("mountain.urdf", mountain_position, mountain_orientation, useFixedBase=1)
    # mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position, mountain_orientation, useFixedBase=1)

    mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)

    # # generate a random creature
    # cr = creature.Creature(gene_count=3)
    # dna = genome.Genome.from_csv(csv_file)
    # cr.update_dna(dna)
    # # save it to XML
    # with open('test.urdf', 'w') as f:
    #     f.write(cr.to_xml())
    # # load it into the sim
    # rob1 = p.loadURDF('test.urdf', (7, 7, 1))

    # start_pos, orn = p.getBasePositionAndOrientation(rob1)

    pop = population.Population(pop_size=10, gene_count=3)
    sim = simulation.Simulation()
    # Initialize new_data before your main loop
    new_data = []

    for iteration in range(5):
        for cr in pop.creatures:
            sim.run_creature(cr, 2400)            
        fits = [cr.get_distance_travelled() for cr in pop.creatures]
        links = [len(cr.get_expanded_links()) for cr in pop.creatures]
        print(iteration, "fittest:", np.round(np.max(fits), 3), 
            "mean:", np.round(np.mean(fits), 3), "mean links", np.round(np.mean(links)), "max links", np.round(np.max(links))) 
        # Store the data for this generation in a dictionary
        generation_data = {
            "iteration": iteration,
            "fittest": np.round(np.max(fits), 3),
            "mean": np.round(np.mean(fits), 3),
            "mean links": np.round(np.mean(links)),
            "max links": np.round(np.max(links))
        }

        # Add the dictionary to the new list
        new_data.append(generation_data)
      
        fit_map = population.Population.get_fitness_map(fits)
        new_creatures = []
        for i in range(len(pop.creatures)):
            p1_ind = Population.select_parent(fit_map)
            p2_ind = Population.select_parent(fit_map)
            p1 = pop.creatures[p1_ind]
            p2 = pop.creatures[p2_ind]
            dna = Genome.crossover(p1.dna, p2.dna)
            dna = Genome.point_mutate(dna, rate=0.1, amount=0.25)
            dna = Genome.shrink_mutate(dna, rate=0.25)
            dna = Genome.grow_mutate(dna, rate=0.1)
            cr = Creature(1)
            cr.update_dna(dna)
            new_creatures.append(cr)
        # elitism
        max_fit = np.max(fits)
        for cr in pop.creatures:
            if cr.get_distance_travelled() == max_fit:
                new_cr = Creature(1)
                new_cr.update_dna(cr.dna)
                new_creatures[0] = new_cr
                filename = "elite_" + str(iteration) + ".csv"
                try:
                    Genome.to_csv(cr.dna, filename)
                    print(f"CSV file created: {filename}")
                except Exception as e:
                    print(f"Error creating CSV file: {e}")
                break
        else:
            print("No creature matched the max_fit condition.")
    # At the end of your script, outside the loop
    # Convert the new list of dictionaries to a DataFrame
    new_df = pd.DataFrame(new_data)

    # Save the new DataFrame to a CSV file with a different name
    new_df.to_csv("new_ga_output.csv", index=False)

    # Plotting the graph
    plt.figure(figsize=(10, 6))
    plt.plot(new_df['iteration'], new_df['fittest'])
    plt.xlabel('Iteration')
    plt.ylabel('Fittest')
    plt.title('Fittest value over iterations')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()




