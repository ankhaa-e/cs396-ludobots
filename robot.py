from motor import MOTOR
from sensor import SENSOR

import constants as c
import pybullet as p
import pyrosim.pyrosim as pyrosim

class ROBOT:
    def __init__(self):
        self.motors = {}
        self.sensors = {}

        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.phaseOffset = c.phaseOffset

        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)
    
    def Prepare_To_Act(self):
     
        for jointName in pyrosim.jointNamesToIndices:
            mod = 1 if jointName == b'Torso_BackLeg' else .5
            self.motors[jointName] = MOTOR(jointName, self.robotId,
                    self.amplitude,mod * self.frequency,self.phaseOffset)

    def Sense(self,i):
        for sensor in self.sensors.values():
            sensor.Get_Value(i)

    def Act(self,i):
        for motor in self.motors.values():
            motor.Set_Value(i)