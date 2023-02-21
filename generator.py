import pyrosim.pyrosim as pyrosim
import constants as c
import random

class Generator:
    def __init__(self):
        self.sensors = []
        self.joints = []
        self.numBodyParts = 0

    def Create_World():
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[0-2,0+2,1-1] , size=[1,1,1])
        pyrosim.End()

    def Randomize_Dimensions(self, xScale=1,yScale=1, zScale=1):
        #length, width, height
        return xScale * (random.random() + 1), yScale * (random.random() +.75), zScale * (random.random() +.5)
    
    def IsSensor(self, currSegment):
        if random.random() >= .67:
            self.sensors.append(currSegment)
            return "0 1.0 0.5 1.0"
        return "0 1.0 1.0 1.0"
    
    def Create_Robot(self):
        while len(self.sensors) == 0:
            self.sensors = []
            self.joints = []
            self.Generate_Body()
        self.Generate_Brain()

    
    def Generate_Limb(self, parentLength,parentWidth,parentHeight, parentName, xSideMultiplier, ySideMultiplier=0):
        

        if ySideMultiplier == 0:
            length, width, height = self.Randomize_Dimensions(.6*parentLength,.4*parentWidth,.4*parentHeight)   
            depth = True if random.random() >= .5 else False
            
            for yMult in [1,-1]:
                self.numBodyParts += 1
                currSegment = str(self.numBodyParts)
                if depth:
                    color = self.IsSensor(currSegment)
                else:
                    color = "0 1.0 0.5 1.0"
                    self.sensors.append(currSegment)
                z= 2 if parentName == "1" else 0
                pyrosim.Send_Joint( name = parentName+"_"+currSegment , parent= parentName , child = currSegment,
                                    type = "revolute", position = [xSideMultiplier*parentLength/2,yMult*.5*parentWidth,z-.25 *parentHeight], jointAxis="1 1 0")
                pyrosim.Send_Cube(name=currSegment, pos=[0,yMult*.5*width,0], size=[length,width,height],color=color)
                self.joints.append(parentName+"_"+currSegment)
                
                if depth:
                    self.Generate_Limb(length,width,height, currSegment, xSideMultiplier, yMult)

        else:
            length, width, height = parentLength*.2, parentWidth, parentHeight*.5
            self.numBodyParts += 1
            currSegment = str(self.numBodyParts)
            color = "0 1.0 0.5 1.0"
            self.sensors.append(currSegment)
            z= 0
            rotation = str(-1*ySideMultiplier * 45)+ " 0 0"
            pyrosim.Send_Joint( name = parentName+"_"+currSegment , parent= parentName , child = currSegment,
                                type = "revolute", position = [0,ySideMultiplier*parentWidth,z-.25 *parentHeight], jointAxis="1 1 0",rotation=rotation)
            pyrosim.Send_Cube(name=currSegment, pos=[0,ySideMultiplier*.5*width,0], size=[length,width,height],color=color)
            self.joints.append(parentName+"_"+currSegment)
            '''
            pyrosim.Send_Joint( name = "Head_LFLeg1" , parent= "Head" , child = "LFLeg1" , type = "revolute", position = [-.65,0.2,0], jointAxis="1 0 -1",rotation="0 0 45")
            pyrosim.Send_Cube(name="LFLeg1", pos=[0,0.375,0] , size=[.2*length,width*.75,.2*height])
            pyrosim.Send_Joint( name = "LFLeg1_LFLeg2" , parent= "LFLeg1" , child = "LFLeg2" , type = "revolute", position = [0,.75,0],jointAxis="1 0 -1")
            pyrosim.Send_Cube(name="LFLeg2", pos=[0,0,-.375] , size=[.2*length,.2*width,height*.75])
            '''
            
    def Generate_Body(self):
        
        length, width, height = self.Randomize_Dimensions()

        x = 0
        y = 0
        z = 2

        pyrosim.Start_URDF("body.urdf")
        color = self.IsSensor("1")
        pyrosim.Send_Cube(name="1", pos=[x,y,z] , size=[length,width,height],color=color)
        self.numBodyParts = 1
        leftMostSegment = "1"
        leftMostLength = length/2
        rightMostSegment = "1"
        rightMostLength = length/2
        if random.random() >= .5:
            self.Generate_Limb(length,width,height, "1", 0)

        newSegment = True
        '''
        color = self.IsSensor("2")
        pyrosim.Send_Joint( name = "1_2" , parent= "1" , child = "2",
                            type = "revolute", position = [-1*parentLength/2,y,z], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="2", pos=[-1*.5*length,0,0], size=[length,width,height],color=color)
        leftMostSegment = "2"
        leftMostLength = length
        '''
        while(newSegment):
            
            length, width, height = self.Randomize_Dimensions(1,.75,.4)  

            self.numBodyParts += 1
            currSegment = str(self.numBodyParts)
            addToLeft = False#random.random() >= .5
            sideMultiplier = -1 if addToLeft else 1
            parentSegment = leftMostSegment if addToLeft else rightMostSegment 
            parentLength = leftMostLength if addToLeft else rightMostLength
            rotX = 0 if random.random() <= .75 else random.random() * .5
            rotY = 0 if random.random() <= .75 else random.random() * .5
            rotZ = 0 if random.random() <= .1 else random.random() * .1
            rotation = str(rotX) + " " + str(rotY) + " " + str(rotZ)
            z = 2 if parentSegment == "1" else 0
            pyrosim.Send_Joint( name = parentSegment + "_" + currSegment , parent= parentSegment , child = currSegment,
                                type = "revolute", position = [sideMultiplier*parentLength,0,z],jointAxis="0 1 0", rotation=rotation)
            self.joints.append(parentSegment + "_" + currSegment)
            color = self.IsSensor(currSegment)      
            pyrosim.Send_Cube(name=currSegment, pos=[sideMultiplier*length/2,0,0], size=[length,width,height], color=color)
            if addToLeft:
                leftMostLength = length
                leftMostSegment = currSegment
            else:
                rightMostLength = length
                rightMostSegment = currSegment
            if random.random() >= .5:
                self.Generate_Limb(length,width,height, currSegment, sideMultiplier)
            
            if self.numBodyParts == c.sizeLimit: break
            newSegment = random.random() >= .5
        pyrosim.End()

    def Generate_Brain(self):
        print("sensors: ", self.sensors)
        print("joints: ", self.joints)
        pyrosim.Start_NeuralNetwork("brain.nndf")
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
                r = random.random() *2 - 1
                pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j , weight = r )

        pyrosim.End()