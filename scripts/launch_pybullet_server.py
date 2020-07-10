#!/usr/bin/python

import pybullet as p
import time
import numpy as np
import math

import signal

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

# # # Use this is it is the responsibility of "pybullet" to spawn the robot
# kukaUid = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/catkin_py_ws/src/py-flex-assembly/gym_flexassembly/data/kuka-iiwa-7/model.urdf", useFixedBase=True)
# p.resetBasePositionAndOrientation(kukaUid, [0, -0.2, 0.5], [0,0,0,1])



# kuka_id = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/catkin_py_ws/src/py-flex-assembly/gym_flexassembly/data/kuka-iiwa-7-egp-40/model.urdf", useFixedBase=True, flags = p.URDF_USE_INERTIA_FROM_FILE)
# print("KUKA ID = " + str(kuka_id))

# # Workpiece to be polished
# workpiece_id = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/catkin_py_ws/src/py-flex-assembly/gym_flexassembly/data/3d/bend_wood.urdf", useFixedBase=False, flags = p.URDF_USE_INERTIA_FROM_FILE)
# wood_offset_table_x = 0.7
# wood_offset_table_y = -0.2
# wood_offset_table_z = 0.45
# wood_offset_world = [wood_offset_table_x, wood_offset_table_y, wood_offset_table_z]
# p.resetBasePositionAndOrientation(workpiece_id, wood_offset_world, [0,0,1,1])

try:
    print("Waiting for CTRL-C")
    signal.pause()
except (KeyboardInterrupt, SystemExit):
    print("Shutting down...")