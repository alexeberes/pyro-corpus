import os

import pybullet as pblt
import pybullet_data
import time
import numpy as np

import pyrosim_modded.pyrosim_modded as pyrosim

physicsClient = pblt.connect(pblt.GUI)
pblt.configureDebugVisualizer(pblt.COV_ENABLE_GUI,0)
pblt.setAdditionalSearchPath(pybullet_data.getDataPath())

pblt.setGravity(0,0,-9.8)

plane_id = pblt.loadURDF("plane.urdf")

robot_id = pblt.loadURDF("recursive_body_test.urdf")

num_iterations = 1000

pyrosim.Prepare_To_Simulate(robot_id)

for i in range(num_iterations):
    pblt.stepSimulation()
    time.sleep(1/60)

pblt.disconnect()