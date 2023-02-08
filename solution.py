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

        sensors = ["RFLeg2","LFLeg2","LBLeg2","RBLeg2"]
        joints = ["Torso_Abdomen","Torso_Head",
                "Abdomen_LBLeg1","LBLeg1_LBLeg2","Abdomen_RBLeg1","RBLeg1_RBLeg2","Head_LFLeg1","LFLeg1_LFLeg2",
                "Head_RFLeg1","RFLeg1_RFLeg2"]
        n = 0
        for sensor in sensors:
            pyrosim.Send_Sensor_Neuron(name = n , linkName = sensor)
            n+=1
        s = n
        for joint in joints:
            pyrosim.Send_Motor_Neuron( name = n , jointName = joint)
            n+=1
        for i in range(s):
            for j in range(s,n):
                pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j , weight = self.weights[i][j-s] )

        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        #length, width , height

        #body
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z] , size=[.75,.4,.3])
        pyrosim.Send_Joint( name = "Torso_Abdomen" , parent= "Torso" , child = "Abdomen" , type = "revolute", position = [.375,0,1],jointAxis="0 1 0")
        pyrosim.Send_Cube(name="Abdomen", pos=[.375,0,0] , size=[.75,.4,.3])
        pyrosim.Send_Joint( name = "Torso_Head" , parent= "Torso" , child = "Head" , type = "revolute", position = [-.375,0,1],jointAxis="0 1 0")
        pyrosim.Send_Cube(name="Head", pos=[-.375,0,0] , size=[.75,.4,.3])

        #pyrosim.Send_Joint( name = "Torso_LMLeg1" , parent= "Torso" , child = "LMLeg1" , type = "revolute", position = [0,0.2,1], jointAxis="1 0 1")
        #pyrosim.Send_Cube(name="LMLeg1", pos=[0,0.375,0] , size=[.2*length,width*.75,.2*height])
        #pyrosim.Send_Joint( name = "LMLeg1_LMLeg2" , parent= "LMLeg1" , child = "LMLeg2" , type = "revolute", position = [0,.75,0],jointAxis="1 0 0")
        #pyrosim.Send_Cube(name="LMLeg2", pos=[0,0,-.375] , size=[.2*length,.2*width,height*.75])

        #pyrosim.Send_Joint( name = "Torso_RMLeg1" , parent= "Torso" , child = "RMLeg1" , type = "revolute", position = [0,-.2,1], jointAxis="1 0 1")
        #pyrosim.Send_Cube(name="RMLeg1", pos=[0,-.375,0] , size=[.2*length,width*.75,.2*height])
        #pyrosim.Send_Joint( name = "RMLeg1_RMLeg2" , parent= "RMLeg1" , child = "RMLeg2" , type = "revolute", position = [0,-.75,0],jointAxis="1 0 0")
        #pyrosim.Send_Cube(name="RMLeg2", pos=[0,0,-.375] , size=[.2*length,.2*width,height*.75])

        pyrosim.Send_Joint( name = "Abdomen_LBLeg1" , parent= "Abdomen" , child = "LBLeg1" , type = "revolute", position = [.65,0.2,0], jointAxis="1 0 -1",rotation="0 0 -45")
        #TO DO TRY JOINT AXIS 1 0 1
        pyrosim.Send_Cube(name="LBLeg1", pos=[0,0.375,0] , size=[.2*length,width*.75,.2*height])
        pyrosim.Send_Joint( name = "LBLeg1_LBLeg2" , parent= "LBLeg1" , child = "LBLeg2" , type = "revolute", position = [0,.75,0],jointAxis="1 0 -1")
        pyrosim.Send_Cube(name="LBLeg2", pos=[0,0,-.375] , size=[.2*length,.2*width,height*.75])

        pyrosim.Send_Joint( name = "Abdomen_RBLeg1" , parent= "Abdomen" , child = "RBLeg1" , type = "revolute", position = [.65,-.2,0], jointAxis="1 0 1", rotation="0 0 45")
        pyrosim.Send_Cube(name="RBLeg1", pos=[0,-.375,0] , size=[.2*length,width*.75,.2*height])
        pyrosim.Send_Joint( name = "RBLeg1_RBLeg2" , parent= "RBLeg1" , child = "RBLeg2" , type = "revolute", position = [0,-.75,0],jointAxis="1 0 1")
        pyrosim.Send_Cube(name="RBLeg2", pos=[0,0,-.375] , size=[.2*length,.2*width,height*.75])

        pyrosim.Send_Joint( name = "Head_LFLeg1" , parent= "Head" , child = "LFLeg1" , type = "revolute", position = [-.65,0.2,0], jointAxis="1 0 -1",rotation="0 0 45")
        pyrosim.Send_Cube(name="LFLeg1", pos=[0,0.375,0] , size=[.2*length,width*.75,.2*height])
        pyrosim.Send_Joint( name = "LFLeg1_LFLeg2" , parent= "LFLeg1" , child = "LFLeg2" , type = "revolute", position = [0,.75,0],jointAxis="1 0 -1")
        pyrosim.Send_Cube(name="LFLeg2", pos=[0,0,-.375] , size=[.2*length,.2*width,height*.75])

        pyrosim.Send_Joint( name = "Head_RFLeg1" , parent= "Head" , child = "RFLeg1" , type = "revolute", position = [-.65,-.2,0], jointAxis="1 0 1",rotation="0 0 -45")
        pyrosim.Send_Cube(name="RFLeg1", pos=[0,-.375,0] , size=[.2*length,width*.75,.2*height])
        pyrosim.Send_Joint( name = "RFLeg1_RFLeg2" , parent= "RFLeg1" , child = "RFLeg2" , type = "revolute", position = [0,-.75,0],jointAxis="1 0 1")
        pyrosim.Send_Cube(name="RFLeg2", pos=[0,0,-.375] , size=[.2*length,.2*width,height*.75])  
            
        pyrosim.End()