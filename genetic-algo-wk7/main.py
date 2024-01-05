import pybullet as p

rob1 = p.loadURDF('105.urdf')
p.setRealTimeSimulation(1)
mode = p.VELOCITY_CONTROL
p.setJointMotorControl2(rob1, 0, controlMode=mode, targetVelocity=1)
