import numpy as np
import os
import time
import constants as c
import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1
x = 0
y = 0
z =1

class SOLUTION:
    def __init__(self,id):
        self.id = id
        if self.id == 0:
            self.Create_World()
            self.Generate_Body()
        self.weights = np.random.rand(c.numSensorNeurons,c.numMotorNeurons)*2 -1
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
        path = "fitness" + str(self.id) + ".txt"
        while not os.path.exists(path):
            time.sleep(0.1)
        fitnessFile = open(path,"r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()
        os.remove(path)



    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        #pyrosim.Send_Cube(name="Box", pos=[x-2,y+2,z-1] , size=[length,width,height])
        pyrosim.End()
        

    def Generate_Bodyold(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[length,width,height])
        
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0,-0.5,1],jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0] , size=[.2*length,width,.2*height])
        pyrosim.Send_Joint( name = "BackLeg_BackLeg2" , parent= "BackLeg" , child = "BackLeg2" , type = "revolute", position = [0,-1,0],jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackLeg2", pos=[0,0,-.5] , size=[.2*length,.2*width,height])

        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0,0.5,1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0] , size=[.2*length,width,.2*height])
        pyrosim.Send_Joint( name = "FrontLeg_FrontLeg2" , parent= "FrontLeg" , child = "FrontLeg2" , type = "revolute", position = [0,1,0],jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontLeg2", pos=[0,0,-.5] , size=[.2*length,.2*width,height])


        pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5,0,1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0] , size=[length,.2*width,.2*height])
        pyrosim.Send_Joint( name = "LeftLeg_LeftLeg2" , parent= "LeftLeg" , child = "LeftLeg2" , type = "revolute", position = [-1,0,0],jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLeg2", pos=[0,0,-.5] , size=[.2*length,.2*width,height])

        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5,0,1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0] , size=[length,.2*width,.2*height])
        pyrosim.Send_Joint( name = "RightLeg_RightLeg2" , parent= "RightLeg" , child = "RightLeg2" , type = "revolute", position = [1,0,0],jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLeg2", pos=[0,0,-.5] , size=[.2*length,.2*width,height])

        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain"+ str(self.id) +".nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "BackLeg2")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg2") 
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "LeftLeg2") 
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "RightLeg2") 

        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_BackLeg")  
        pyrosim.Send_Motor_Neuron( name = 5 , jointName = "Torso_FrontLeg")  
        pyrosim.Send_Motor_Neuron( name = 6 , jointName = "Torso_LeftLeg")  
        pyrosim.Send_Motor_Neuron( name = 7 , jointName = "Torso_RightLeg")

        pyrosim.Send_Motor_Neuron( name = 8 , jointName = "BackLeg_BackLeg2")  
        pyrosim.Send_Motor_Neuron( name = 9 , jointName = "FrontLeg_FrontLeg2")  
        pyrosim.Send_Motor_Neuron( name = 10 , jointName = "LeftLeg_LeftLeg2")  
        pyrosim.Send_Motor_Neuron( name = 11 , jointName = "RightLeg_RightLeg2")
        for i in range(c.numSensorNeurons):
            for j in range(c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j+c.numSensorNeurons , weight = self.weights[i][j] )
        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        #length, width , height
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[.75,.5,.4])
        pyrosim.Send_Joint( name = "Torso_Abdomen" , parent= "Torso" , child = "Abdomen" , type = "revolute", position = [0,0,.75],jointAxis="1 0 0")
        pyrosim.Send_Cube(name="Abdomen", pos=[0,.25,0] , size=[.5,.6,.5])
        pyrosim.End()