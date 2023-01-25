import numpy as np
import os
import time
import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1
x = 0
y = 0
z =1.5

class SOLUTION:
    def __init__(self,id):
        self.id = id
        self.Create_World()
        self.Generate_Body()
        self.weights = np.random.rand(3,2)*2 -1
        self.fitness = 0

    def Set_ID(self, id):
        self.id = id

    def Start_Simulation(self,showSim=False):
        self.Generate_Brain()
        if showSim:
            os.system("START /B py simulate.py GUI " + str(self.id))
        else:
            os.system("START /B py simulate.py DIRECT " + str(self.id))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.id) + ".txt"):
            time.sleep(0.01)
        fitnessFile = open("fitness" + str(self.id) + ".txt","r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()
        os.remove("fitness" + str(self.id) + ".txt")



    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[x-2,y+2,z-1] , size=[length,width,height])
        pyrosim.End()
        

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length,width,height])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [-.5,0,1])
        pyrosim.Send_Cube(name="BackLeg", pos=[-.5,0,-.5] , size=[length,width,height])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [.5,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[.5,0,-.5] , size=[length,width,height])
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain"+ str(self.id) +".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg") 
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")  
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")  
        for i in range(3):
            for j in range(2):
                pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j+3 , weight = self.weights[i][j] )

        pyrosim.End()
