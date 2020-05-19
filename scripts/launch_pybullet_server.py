#!/usr/bin/python

import pybullet as p
import time
import numpy as np
import math

# Setup the GUI and Shared Memory connection
client = p.connect(p.GUI_SERVER)
p.setGravity(0, 0, -9.81, physicsClientId=client)

# Load additional assets
import pybullet_data
print("pybullet_data.getDataPath() = " + str(pybullet_data.getDataPath()))
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create a plane (important for gravity and collisions)
planeId = p.loadURDF("plane.urdf")

# Create some debug sliders
dv = 2.0
dp_joint_pos_0 = p.addUserDebugParameter("j0", -dv, dv, 0)
dp_joint_pos_1 = p.addUserDebugParameter("j1", -dv, dv, 0)
dp_joint_pos_2 = p.addUserDebugParameter("j2", -dv, dv, 0)
dp_joint_pos_3 = p.addUserDebugParameter("j3", -dv, dv, 0)
dp_joint_pos_4 = p.addUserDebugParameter("j4", -dv, dv, 0)
dp_joint_pos_5 = p.addUserDebugParameter("j5", -dv, dv, 0)
dp_joint_pos_6 = p.addUserDebugParameter("j6", -dv, dv, 0)

# Disable realtime
p.setRealTimeSimulation(0)

# Chose step width
p.setTimeStep(0.001) # TODO Chose the right number that matches with OROCOS RTT

while 1:
    joint_pos_0 = p.readUserDebugParameter(dp_joint_pos_0)
    joint_pos_1 = p.readUserDebugParameter(dp_joint_pos_1)
    joint_pos_2 = p.readUserDebugParameter(dp_joint_pos_2)
    joint_pos_3 = p.readUserDebugParameter(dp_joint_pos_3)
    joint_pos_4 = p.readUserDebugParameter(dp_joint_pos_4)
    joint_pos_5 = p.readUserDebugParameter(dp_joint_pos_5)
    joint_pos_6 = p.readUserDebugParameter(dp_joint_pos_6)
