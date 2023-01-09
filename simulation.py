from robot import ROBOT
from world import WORLD

import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
import time

import constants as c

class SIMULATION:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(*c.gravity)

        self.world = WORLD()
        self.robot = ROBOT()

    def __del__(self):
        p.disconnect()

    def Run(self):
        for i in range(c.timesteps):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think(i)
            self.robot.Act(i)
            time.sleep(c.timesleep)

        