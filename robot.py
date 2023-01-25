from motor import MOTOR
from sensor import SENSOR
from pyrosim.neuralNetwork import NEURAL_NETWORK

import constants as c
import pybullet as p
import os
import pyrosim.pyrosim as pyrosim

class ROBOT:
    def __init__(self, simulationId):
        self.motors = {}
        self.sensors = {}
        self.simulationId = simulationId
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.phaseOffset = c.phaseOffset

        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("brain"+ str(simulationId)+ ".nndf")
        os.system("del brain"+ str(simulationId)+ ".nndf")

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

    def Think(self):
        self.nn.Update()

    def Act(self,i):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[jointName.encode('ascii')].Set_Value(desiredAngle)

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        f = open("tmp"+ str(self.simulationId) + ".txt", "w")
        f.write(str(xCoordinateOfLinkZero))
        f.close()
        os.system("rename tmp"+ str(self.simulationId)+".txt fitness"+str(self.simulationId)+".txt")


    