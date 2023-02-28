import random
from bodyJoint import BodyJoint
import pyrosim.pyrosim as pyrosim

class BodyPart:
    def __init__(self, id,x,y,z,length,width,height,sideMultiplier):
        self.id = str(id)
        self.x = x
        self.y = y
        self.z = z
        self.length = length
        self.width = width
        self.height = height
        self.limbs = {}
        self.leftChildren = {}
        self.rightChildren = {}
        self.numChildren = 0
        self.sideMultiplier = sideMultiplier
        self.sensor = random.random() >= .67
        self.color = "0 1.0 0.5 1.0" if self.sensor else "0 1.0 1.0 1.0"


    def Grow(self,addToLeft,length,width,height,numBodyParts):
        numBodyParts += 1
        self.numChildren+=1
        currSegment = str(numBodyParts)
        rotX = 0 if random.random() <= .75 else random.random() * .5
        rotY = 0 if random.random() <= .75 else random.random() * .5
        rotZ = 0 if random.random() <= .1 else random.random() * .1
        rotation = str(rotX) + " " + str(rotY) + " " + str(rotZ)
        sideMultiplier = -1 if addToLeft else 1
        z = 2 if self.id == "1" else 0
        control = .5 if self.id == "1" else 1
        bodyJoint = BodyJoint(self.id,str(numBodyParts), 
                                sideMultiplier*self.length*control,
                                0,
                                z,
                                "0 1 0",
                                rotation=rotation
                                )
        bodySegment =  BodyPart(currSegment,
                                    sideMultiplier*length/2, 0,0,
                                    length, width, height,
                                    sideMultiplier
                                )
        side = self.leftChildren if addToLeft else self.rightChildren
        side.update({bodyJoint:bodySegment})
        return bodySegment

    def Generate_Limb(self,length,width,height, numBodyParts):
               
            depth = True if random.random() >= .5 else False
            
            for yMult in [1,-1]:
                numBodyParts += 1
                self.numChildren+=1
                currSegment = str(numBodyParts)

                z= 2 if self.id == "1" else 0
                """pyrosim.Send_Joint( name = parentName+"_"+currSegment , parent= parentName , child = currSegment,
                                    type = "revolute", position = [xSideMultiplier*parentLength/2,yMult*.5*parentWidth,z-.25 *parentHeight], jointAxis="1 1 0")
                pyrosim.Send_Cube(name=currSegment, pos=[0,yMult*.5*width,0], size=[length,width,height],color=color)"""
                limbJoint = BodyJoint(self.id,currSegment, 
                                             self.sideMultiplier*self.length*.5,
                                             yMult*.5*self.width,
                                             z-.25 *self.height,
                                             "1 1 0"
                                     )
                limbSegment =  BodyPart(currSegment,
                                            0, yMult*.5*width,0,
                                            length, width, height,
                                            0
                                       )
                self.limbs.update({limbJoint: limbSegment})
                if depth:         
                    numBodyParts += 1
                    self.numChildren+=1
                    limbSegment.Generate_SecondLimb(length,width,height,str(numBodyParts), yMult)
                else:
                    limbSegment.color = "0 1.0 0.5 1.0"
                    limbSegment.sensor = True


    def Generate_SecondLimb(self,length,width,height, currSegment, yMult):
        self.numChildren+=1
        currSegment = str(int(self.id)+1)
        rotation = str(-1*yMult * 45)+ " 0 0"
        limbJoint = BodyJoint(self.id,currSegment, 
                                        0,
                                        self.width*yMult,
                                        -.25 *self.height,
                                        "1 1 0",
                                        rotation
                                )
        limbSegment =  BodyPart(currSegment,
                                    0, yMult*.5*width,0,
                                    length, width, height,
                                    0
                                )
        limbSegment.color = "0 1.0 0.5 1.0"
        limbSegment.sensor = True
        self.limbs.update({limbJoint: limbSegment})

    def Get_NewParts(self):
        sensors = []
        joints = []
        
        if self.sensor: sensors.append(self.id)
        for joint, limbSegment in self.limbs.items():
            joints.append(joint.name)
            newSensors, newJoints = limbSegment.Get_NewParts()
            sensors+=newSensors
            joints+=newJoints
        return sensors, joints

    def Write_Body(self):
        sensors = []
        joints = []
        pyrosim.Send_Cube(name=self.id, pos=[self.x,self.y,self.z] , size=[self.length,self.width,self.height],color=self.color)
        if self.sensor: sensors.append(self.id)
        for joint, limbSegment in self.limbs.items():
            pyrosim.Send_Joint( name =joint.name , parent= joint.parentName , child = joint.childName,
                                type = "revolute", position = [joint.x,joint.y,joint.z], jointAxis=joint.axis)
            joints.append(joint.name)
            newSensors, newJoints = limbSegment.Write_Body()
            sensors += newSensors
            joints += newJoints
        for joint, bodySegment in self.leftChildren.items():
            pyrosim.Send_Joint( name =joint.name , parent= joint.parentName , child = joint.childName,
                                type = "revolute", position = [joint.x,joint.y,joint.z], jointAxis=joint.axis)
            joints.append(joint.name)
            newSensors, newJoints = bodySegment.Write_Body()
            sensors += newSensors
            joints += newJoints  
        for joint, bodySegment in self.rightChildren.items():
            pyrosim.Send_Joint( name =joint.name , parent= joint.parentName , child = joint.childName,
                                type = "revolute", position = [joint.x,joint.y,joint.z], jointAxis=joint.axis)
            joints.append(joint.name)
            newSensors, newJoints = bodySegment.Write_Body()
            sensors += newSensors
            joints += newJoints

        return sensors,joints