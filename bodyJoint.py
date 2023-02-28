
class BodyJoint:
    def __init__(self, parentName, childName,x,y,z,axis,rotation= "0 0 0"):
        self.parentName = parentName
        self.childName = childName
        self.name = parentName + "_" + self.childName
        self.x = x
        self.y = y 
        self.z = z
        self.axis = axis
        self.rotation = rotation

