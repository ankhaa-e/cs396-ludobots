import pyrosim.pyrosim as pyrosim
import constants as c
import random
import math
from generator import Generator
length = 1
width = 1
height = 1

x = 0
y = 0
z =1


def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[x-2,y+2,z-1] , size=[length,width,height])
    pyrosim.End()

def Create_Robot():
    sensors = []
    joints = []
    sensors, joints = Generate_Snake(sensors,joints)
    Generate_SnakeBrain(sensors,joints)

def Generate_Body():
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

def Randomize_Dimensions():
    #length, width, height
    return 1.5 * (random.random() +.1), 1.5 * (random.random() +.1), 1.5 * (random.random() +.1)
def IsSensor(currSegment, sensors):
    if random.random() >= .67:
        sensors.append(currSegment)
        return "0 1.0 0.5 1.0", sensors
    return "0 1.0 1.0 1.0", sensors
def Generate_Creature(joints,sensors):
    
    length, width, height = Randomize_Dimensions()

    x = 0
    y = 0
    z = 1

    pyrosim.Start_URDF("body.urdf")
    color, sensors = IsSensor("1",sensors)
    pyrosim.Send_Cube(name="1", pos=[x,y,z] , size=[length,width,height],color=color)

    parentLength = length


    length, width, height = Randomize_Dimensions()    
    color, sensors = IsSensor("2",sensors)
    pyrosim.Send_Joint( name = "1_2" , parent= "1" , child = "2",
                        type = "revolute", position = [-1*parentLength/2,y,z], jointAxis="0 1 0")
    pyrosim.Send_Cube(name="2", pos=[-1*.5*length,0,0], size=[length,width,height],color=color)
    leftMostSegment = "2"
    leftMostLength = length
    
    length, width, height = Randomize_Dimensions()
    color, sensors = IsSensor("3",sensors)
    pyrosim.Send_Joint( name = "1_3" , parent= "1" , child = "3",
                        type = "revolute", position = [parentLength/2,y,z],jointAxis="0 1 0")
    pyrosim.Send_Cube(name="3", pos=[.5*length,0,0], size=[length,width,height],color=color)

    rightMostSegment = "3"
    rightMostLength = length
    
    numBodyParts = 3
    newSegment = True
    joints += ["1_2","1_3"]

    while(newSegment):
        
        length, width, height = Randomize_Dimensions()  

        numBodyParts += 1
        currSegment = str(numBodyParts)
        addToLeft = random.random() >= .5
        sideMultiplier = -1 if addToLeft else 1
        parentSegment = leftMostSegment if addToLeft else rightMostSegment 
        parentLength = leftMostLength if addToLeft else rightMostLength
        rotX = 0 if random.random() <= .75 else random.randint(15,45)
        rotY = 0 #if random.random() <= .75 else random.randint(15,45)
        rotZ = 0 if random.random() <= .75 else random.randint(15,45)
        rotation = str(rotX) + " " + str(rotY) + " " + str(rotZ)

        pyrosim.Send_Joint( name = parentSegment + "_" + currSegment , parent= parentSegment , child = currSegment,
                            type = "revolute", position = [sideMultiplier*parentLength,0,0],jointAxis="0 1 0", rotation=rotation)
        joints.append(parentSegment + "_" + currSegment)
        color,sensors = IsSensor(currSegment,sensors)      
        pyrosim.Send_Cube(name=currSegment, pos=[sideMultiplier*length*.5,0,0], size=[length,width,height], color=color)
        if addToLeft:
            leftMostLength = length
            leftMostSegment = currSegment
        else:
            rightMostLength = length
            rightMostSegment = currSegment

        if numBodyParts == c.sizeLimit: break
        newSegment = random.random() >= .25
    pyrosim.End()
    return sensors, joints
def Generate_Snake(joints,sensors):
    
    length, width, height = Randomize_Dimensions()

    x = 0
    y = 0
    z = 1

    pyrosim.Start_URDF("body.urdf")
    color, sensors = IsSensor("1",sensors)
    pyrosim.Send_Cube(name="1", pos=[x,y,z] , size=[length,width,height],color=color)

    parentLength = length


    length, width, height = Randomize_Dimensions()    
    color, sensors = IsSensor("2",sensors)
    pyrosim.Send_Joint( name = "1_2" , parent= "1" , child = "2",
                        type = "revolute", position = [-1*parentLength/2,y,z], jointAxis="0 1 0")
    pyrosim.Send_Cube(name="2", pos=[-1*.5*length,0,0], size=[length,width,height],color=color)
    leftMostSegment = "2"
    leftMostLength = length
    
    length, width, height = Randomize_Dimensions()
    color, sensors = IsSensor("3",sensors)
    pyrosim.Send_Joint( name = "1_3" , parent= "1" , child = "3",
                        type = "revolute", position = [parentLength/2,y,z],jointAxis="0 1 0")
    pyrosim.Send_Cube(name="3", pos=[.5*length,0,0], size=[length,width,height],color=color)

    rightMostSegment = "3"
    rightMostLength = length
    
    numBodyParts = 3
    newSegment = True
    joints += ["1_2","1_3"]

    while(newSegment):
        
        length, width, height = Randomize_Dimensions()  

        numBodyParts += 1
        currSegment = str(numBodyParts)
        addToLeft = random.random() >= .5
        sideMultiplier = -1 if addToLeft else 1
        parentSegment = leftMostSegment if addToLeft else rightMostSegment 
        parentLength = leftMostLength if addToLeft else rightMostLength
        rotX = 0 if random.random() <= .75 else random.randint(15,45)
        rotY = 0 #if random.random() <= .75 else random.randint(15,45)
        rotZ = 0 if random.random() <= .75 else random.randint(15,45)
        rotation = str(rotX) + " " + str(rotY) + " " + str(rotZ)

        pyrosim.Send_Joint( name = parentSegment + "_" + currSegment , parent= parentSegment , child = currSegment,
                            type = "revolute", position = [sideMultiplier*parentLength,0,0],jointAxis="0 1 0", rotation=rotation)
        joints.append(parentSegment + "_" + currSegment)
        color,sensors = IsSensor(currSegment,sensors)      
        pyrosim.Send_Cube(name=currSegment, pos=[sideMultiplier*length*.5,0,0], size=[length,width,height], color=color)
        if addToLeft:
            leftMostLength = length
            leftMostSegment = currSegment
        else:
            rightMostLength = length
            rightMostSegment = currSegment

        if numBodyParts == c.sizeLimit: break
        newSegment = random.random() >= .25
    pyrosim.End()
    return sensors, joints






def Generate_SnakeBrain(sensors,joints):
    print("sensors: ",sensors)
    print("joints: ",joints)
    pyrosim.Start_NeuralNetwork("brain.nndf")
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
            r = random.random() *2 - 1
            pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j , weight = r )

    pyrosim.End()

def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
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
            r = random.random() *2 - 1
            pyrosim.Send_Synapse( sourceNeuronName = i , targetNeuronName = j , weight = r )

    pyrosim.End()

g= Generator()
#Create_World()
g.Create_Robot()