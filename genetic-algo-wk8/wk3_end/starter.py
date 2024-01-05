import pybullet as p
import pybullet_data as pd
import genome
import creature
import time
import random

p.connect(p.GUI)
p.setPhysicsEngineParameter(enableFileCaching=0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
plane_shape = p.createCollisionShape(p.GEOM_PLANE)
floor = p.createMultiBody(plane_shape, plane_shape)
p.setGravity(0, 0, -10)

cr = creature.Creature(gene_count=3)
# save it to XML
with open('test.urdf', 'w') as f:
    f.write(cr.to_xml())
# load it into the sim
rob1 = p.loadURDF('test.urdf')

step = 0
while True:
    p.stepSimulation()
    step += 1
    if step % 120 == 0:
        motors = cr.get_motors()
        assert len(motors) == p.getNumJoints(rob1), "bad motors!"
        for jid in range(p.getNumJoints(rob1)):
            mode = p.VELOCITY_CONTROL
            vel = 5 * (random.random() - 0.5)
            p.setJointMotorControl2(rob1,
                jid,
                controlMode=mode,
                targetVelocity=vel)
    time.sleep(1.0/240)
