import numpy as np
import os
import time
import constants as c
import pyrosim.pyrosim as pyrosim
import random
from bodyPart import BodyPart

length = 1
width = 1
height = 1
x = 0
y = 0
z =1

class SOLUTION:
    def __init__(self,id):
        self.id = id

        self.segments = []
        self.sensors = []
        self.joints = []

        self.numBodyParts = 0

        self.leftMostSegment = None
        self.rightMostSegment = None
        self.base = None
        self.Generate_Body()
        self.Write_Body()
        #if self.id == 0:
        #    self.Create_World()

        self.fitness = 0
        
        self.weights = np.random.rand(len(self.sensors),len(self.joints))*2 -1
        self.Generate_Brain()

    def Mutate(self,r=random.random()):
        #r = random.random()

        if r <= .25:
            #print("Adding new Segment")
            addToLeft = random.random() >= .5
            parentSegment = self.leftMostSegment if addToLeft else self.rightMostSegment
            length, width, height = self.Randomize_Dimensions(1,.75,.4)
            newPart = parentSegment.Grow(addToLeft,length,width,height,self.numBodyParts)
            self.numBodyParts += 1
            if random.random() >= .67:
                length, width, height = self.Randomize_Dimensions(.6*newPart.length,.4*newPart.width,.4*newPart.height)   
                newPart.Generate_Limb(length,width,height,self.numBodyParts)
            self.numBodyParts += newPart.numChildren

            self.segments.append(newPart)
            newSensors, newJoints = newPart.Get_NewParts()
            newJoint = str(parentSegment.id + "_" + newPart.id)
            newJoints.append(newJoint)
            

            
            self.weights = np.append(self.weights,np.random.rand(len(self.sensors),len(newJoints))*2 -1,axis=1)

            #sensors = np.append(sensors,np.random.rand(len(newJoints))*2 -1,axis=0)
            # add body part
            self.weights = np.vstack([self.weights,np.random.rand(len(newSensors),len(self.joints)+len(newJoints))*2 -1])
            if addToLeft:
                self.leftMostSegment = newPart
            else:
                self.rightMostSegment = newPart

        elif r <= .5:
            #print("Removing Segment")
            removeFromLeft = random.random() >= .5    
            
            currSegment = self.base
            lastSegment = self.base

            removedSensors= []
            removedJoints =[]

            if removeFromLeft:  
                while len(currSegment.leftChildren) > 0:
                    lastSegment = currSegment
                    currSegment = next(iter(currSegment.leftChildren.values()))

                if lastSegment != self.base:
                    lastSegment.leftChildren = {}
                    self.leftMostSegment = lastSegment
            else:
                while len(currSegment.rightChildren) > 0:
                    lastSegment = currSegment
                    currSegment = next(iter(currSegment.rightChildren.values()))
                if lastSegment != self.base:
                    lastSegment.rightChildren = {}
                    self.rightMostSegment = lastSegment

        elif r <= .75:
            changeLeft = random.random() >= .5    
            currSegment = self.leftMostSegment if changeLeft else self.rightMostSegment

            if len(currSegment.limbs) == 0:
                length, width, height = self.Randomize_Dimensions(.6*currSegment.length,.4*currSegment.width,.4*currSegment.height)   
                currSegment.Generate_Limb(length,width,height,self.numBodyParts)
                self.numBodyParts += currSegment.numChildren

                newSensors, newJoints = currSegment.Get_NewParts()
                self.weights = np.append(self.weights,np.random.rand(len(self.sensors),len(newJoints))*2 -1,axis=1)
                self.weights = np.vstack([self.weights,np.random.rand(len(newSensors),len(self.joints)+len(newJoints))*2 -1])
                #print("adding limbs")
            else:    
                #print("removing limbs") 
                currSegment.limbs = {}

            # add/remove limbs

        self.Write_Body()

        #print(i,j)
        while len(self.joints) < len(self.weights[0]):
            self.weights = np.delete(self.weights,-1,1)
        while len(self.sensors) < len(self.weights):
            self.weights = np.delete(self.weights,-1,0)
        i = random.randint(0,len(self.sensors)-1)
        j = random.randint(0,len(self.joints)-1)
        self.weights[i][j] = random.random() * 2 - 1
        self.Generate_Brain()
        #change a random weight

    def Set_ID(self, id):
        self.id = id

    def Write_Files(self):
        self.Generate_Brain()

    def Start_Simulation(self,showSim=False):

        if showSim:
            os.system("START /B py simulate.py GUI " + str(self.id))
        else:
            os.system("START /B py simulate.py DIRECT " + str(self.id))

    def Wait_For_Simulation_To_End(self):
        path = "fitness" + str(self.id) + ".txt"
        while not os.path.exists(path):
            time.sleep(1)
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
        pyrosim.End()



    def Generate_Brain(self):
        #print(self.weights)
        pyrosim.Start_NeuralNetwork("brain"+ str(self.id) +".nndf")
        n = 0
        for sensor in self.sensors:
            pyrosim.Send_Sensor_Neuron(name = n , linkName = sensor)
            n+=1
        s = n
        for joint in self.joints:
            pyrosim.Send_Motor_Neuron( name = n , jointName = joint)
            n+=1
        for i in range(s):
            for j in range(s,n):            
                pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j , weight = self.weights[i][j-s] )

        pyrosim.End()

    def Randomize_Dimensions(self, xScale=1,yScale=1, zScale=1):
        #length, width, height
        return xScale * (random.random() + 1), yScale * (random.random() +.75), zScale * (random.random() +.5)
    
    def IsSensor(self, currSegment):
        if random.random() >= .67:
            self.sensors.append(currSegment)
            return "0 1.0 0.5 1.0"
        return "0 1.0 1.0 1.0"
    
    def Write_Body(self):
        """pyrosim.Start_URDF("body"+ str(self.id) +".urdf")
        self.sensors, self.joints = self.base.Write_Body()
        while len(self.sensors) == 0:
            self.sensors = []
            self.joints = []
            
            self.Generate_Body()
            self.sensors, self.joints = self.base.Write_Body()
        #print("sensors: ", self.sensors)
        #print("joints: ", self.joints)
        pyrosim.End()
        print("made body ", self.id)"""



        pyrosim.Start_URDF("body"+ str(self.id) +".urdf")
        self.sensors, self.joints = self.base.Write_Body()
        pyrosim.End()
        while len(self.sensors) == 0:
            self.sensors = []
            self.joints = []
            self.Generate_Body()
            pyrosim.Start_URDF("body"+ str(self.id) +".urdf")
            self.sensors, self.joints = self.base.Write_Body()
            pyrosim.End()

    def Generate_Body(self):
        length, width, height = self.Randomize_Dimensions()
        x = 0
        y = 0
        z = 2
        self.numBodyParts = 1
        baseBody = BodyPart(str(self.numBodyParts),x,y,z,length,width,height,0)
        if baseBody.sensor: self.sensors.append(baseBody.id)
        self.segments.append(baseBody)
        if random.random() >= .5:
            length, width, height = self.Randomize_Dimensions(.6*baseBody.length,.4*baseBody.width,.4*baseBody.height)   
            baseBody.Generate_Limb(length,width,height,1)
        self.leftMostSegment = baseBody
        self.rightMostSegment = baseBody
        self.numBodyParts += baseBody.numChildren
        newSegment = True
        self.base = baseBody
        while newSegment:
            length, width, height = self.Randomize_Dimensions(1,.75,.4)
            addToLeft = random.random() >= .5
            parentSegment = self.leftMostSegment if addToLeft else self.rightMostSegment
            newPart = parentSegment.Grow(addToLeft,length,width,height,self.numBodyParts)
            self.numBodyParts += 1
            if random.random() >= .67:
                length, width, height = self.Randomize_Dimensions(.6*newPart.length,.4*newPart.width,.4*newPart.height)   
                newPart.Generate_Limb(length,width,height,self.numBodyParts)
            self.numBodyParts += newPart.numChildren

            if addToLeft:
                self.leftMostSegment = newPart
            else:
                self.rightMostSegment = newPart
            self.segments.append(newPart)

            if self.numBodyParts > c.sizeLimit: break
            newSegment = random.random() >= .25

