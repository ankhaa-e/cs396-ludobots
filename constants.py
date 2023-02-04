import math

gravity = [0,0,-9.8]

force = 50
timesleep = 1/240
timesteps = 1000

populationSize = 10
numGenerations = 10

numSensorNeurons = 4
numMotorNeurons = 8

motorJointRange = .4

#FrontLegs
amplitude = math.pi/5
frequency = (math.pi*2)/100
phaseOffset = 0

#BackLegs
amplitude2 = math.pi/3
frequency2 = (math.pi*2)/50
phaseOffset2 = math.pi/4
