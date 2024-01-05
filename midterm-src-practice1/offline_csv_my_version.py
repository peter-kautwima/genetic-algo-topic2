import os 
import genome
import sys
import creature
import pybullet as p
import time 
import random
import numpy as np
import glob
import matplotlib.pyplot as plt

def main(csv_file):
    """
    This function performs some operations on the given CSV file.
    It calculates the total distance moved and returns it.

    Parameters:
    csv_file (str): The path of the CSV file.

    Returns:
    float: The total distance moved.
    """
    assert os.path.exists(csv_file), "Tried to load " + csv_file + " but it does not exists"

    p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    plane_shape = p.createCollisionShape(p.GEOM_PLANE)
    floor = p.createMultiBody(plane_shape, plane_shape)
    p.setGravity(0, 0, -10)
#   p.setRealTimeSimulation(1)


    # generate a random creature
    cr = creature.Creature(gene_count=1)
    dna = genome.Genome.from_csv(csv_file)
    cr.update_dna(dna)
    # save it to XML
    with open('test.urdf', 'w') as f:
        f.write(cr.to_xml())
    # load it into the sim
    rob1 = p.loadURDF('test.urdf')
    # air drop it
    p.resetBasePositionAndOrientation(rob1, [0, 0, 2.5], [0, 0, 0, 1])

    start_pos, orn = p.getBasePositionAndOrientation(rob1)

    # iterate 
    elapsed_time = 0
    wait_time = 1.0/240 # seconds
    total_time = 30 # seconds
    step = 0
    dist_moved = 0
    while True:
        p.stepSimulation()
        step += 1
        if step % 24 == 0:
            motors = cr.get_motors()
            assert len(motors) == p.getNumJoints(rob1), "Something went wrong"
            for jid in range(p.getNumJoints(rob1)):
                mode = p.VELOCITY_CONTROL
                vel = motors[jid].get_output()
                p.setJointMotorControl2(rob1, 
                            jid,  
                            controlMode=mode, 
                            targetVelocity=vel)
            new_pos, orn = p.getBasePositionAndOrientation(rob1)
            #print(new_pos)
            dist_moved = np.linalg.norm(np.asarray(start_pos) - np.asarray(new_pos))
            print(dist_moved)
        #time.sleep(wait_time)
        elapsed_time += wait_time
        if elapsed_time > total_time:
            break

    print("TOTAL DISTANCE MOVED:", dist_moved)
    return dist_moved

def calculate_distances():
    """
    This function calculates the distances for all CSV files in the current directory.
    It calls the main function for each CSV file and stores the distances in a dictionary.

    Returns:
    dict: A dictionary containing the distances for each CSV file.
    """
    distances = {}
    for csv_file in glob.glob('*.csv'):
        distance = main(csv_file)
        distances[csv_file] = distance
    return distances

def plot_distances(distances):
    """
    This function plots a bar chart showing the distances traveled for each CSV file.

    Parameters:
    distances (dict): A dictionary containing the distances for each CSV file.
    """
    plt.bar(distances.keys(), distances.values())
    plt.xlabel('CSV File')
    plt.ylabel('Total Distance Traveled')
    plt.show()

distances = calculate_distances()
plot_distances(distances)
