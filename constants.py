import math

gravity = [0,0,-9.8]

force = 75
timesleep = 1/100000
timesteps = 200

populationSize = 10
numGenerations = 10

numSensorNeurons = 4
numMotorNeurons = 10

motorJointRange = .35

#FrontLegs
amplitude = math.pi/5
frequency = (math.pi*2)/100
phaseOffset = 0

#BackLegs
amplitude2 = math.pi/3
frequency2 = (math.pi*2)/50
phaseOffset2 = math.pi/4

sizeLimit = 10