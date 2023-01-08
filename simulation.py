import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
import time 
import math
import random
import numpy as np
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)
targetFrontAngles = np.zeros(1000)
targetBackAngles = np.zeros(1000)

#FrontLegs
amplitude1 = np.pi/5
frequency1 = (np.pi*2)/100
phaseOffset1 = 0

#BackLegs
amplitude2 = np.pi/3
frequency2 = (np.pi*2)/50
phaseOffset2 = np.pi/4

for i in range(1000):
    targetFrontAngles[i] = amplitude1 * np.sin(frequency1*i + phaseOffset1) 
    targetBackAngles[i] = amplitude2 * np.sin(frequency2*i+phaseOffset2) 

for i in range(1000):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b"Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetBackAngles[i],
        maxForce = 50)
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b"Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetFrontAngles[i],
        maxForce = 50)
    time.sleep(1/240)
    
np.save("data/backLegSensorValues.npy", backLegSensorValues)
np.save("data/frontLegSensorValues.npy", frontLegSensorValues)
p.disconnect()