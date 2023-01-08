import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")


x =0
y = 0
z = .5
for j in range(5):
    for k in range(5):
        length = 1
        width = 1
        height = 1
        for i in range(10):
            pyrosim.Send_Cube(name="Box" + str(i), pos=[x+j,y+k,z+i] , size=[length,width,height])
            length *= .9
            width *= .9
            height *= .9

pyrosim.End()