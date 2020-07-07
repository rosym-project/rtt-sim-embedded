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

kukaUid = p.loadURDF("/home/dwigand/code/cogimon/CoSimA/pyBullet/catkin_py_ws/src/py-flex-assembly/gym_flexassembly/data/kuka-iiwa-7/model.urdf", useFixedBase=True)
p.resetBasePositionAndOrientation(kukaUid, [0, -0.2, 0.5], [0,0,0,1])
# numJoints = p.getNumJoints(kukaUid)
# allJoint = []
# zero = []
# for jon in range(numJoints):
#     allJoint.append(jon)
#     zero.append(0.0)
# p.setJointMotorControlArray(kukaUid, allJoint, p.VELOCITY_CONTROL, forces=zero)

# while 1:
#     # joint_pos_0 = p.readUserDebugParameter(dp_joint_pos_0)
#     # joint_pos_1 = p.readUserDebugParameter(dp_joint_pos_1)
#     # joint_pos_2 = p.readUserDebugParameter(dp_joint_pos_2)
#     # joint_pos_3 = p.readUserDebugParameter(dp_joint_pos_3)
#     # joint_pos_4 = p.readUserDebugParameter(dp_joint_pos_4)
#     # joint_pos_5 = p.readUserDebugParameter(dp_joint_pos_5)
#     # joint_pos_6 = p.readUserDebugParameter(dp_joint_pos_6)
#     print("kukaUid = " + str(kukaUid))

#     indicies = [1,2,3,4,5,6,7]

#     print("numJoints = " + str(numJoints))
#     print("indicies = " + str(len(indicies)))

#     jointStates = p.getJointStates(kukaUid, indicies)
#     q1=[]
#     qdot1=[]
#     zeroAccelerations=[]
#     for i in range(len(indicies)):
#         q1.append(jointStates[i][0])
#         qdot1.append(jointStates[i][1])
#         zeroAccelerations.append(0)
#     print("Pos " + str(q1))
#     print("Vel " + str(qdot1))

#     time.sleep(1/1000) # TODO DLW


try:
    print("Waiting for CTRL-C")
    signal.pause()
except (KeyboardInterrupt, SystemExit):
    print("Shutting down...")