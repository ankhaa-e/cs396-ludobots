from robot import ROBOT
from world import WORLD

import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
import time

import constants as c

class SIMULATION:
    def __init__(self, directOrGUI, simulationId="",rem=True, best=False):
        self.physicsClient = p.connect(p.GUI) if directOrGUI == "GUI" else p.connect(p.DIRECT)
        if directOrGUI: p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(*c.gravity)
        self.direct = directOrGUI != "GUI"
        self.world = WORLD()
        self.robot = ROBOT(simulationId,rem,best)

    def __del__(self):
        p.disconnect()

    def Run(self):
        
        for i in range(c.timesteps):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)
            if not self.direct:
                time.sleep(c.timesleep)

    def Get_Fitness(self):
        self.robot.Get_Fitness()
        