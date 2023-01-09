import constants as c
import numpy as np
import pyrosim.pyrosim as pyrosim

class SENSOR:
    def __init__(self,linkName):
        self.linkName = linkName
        self.values = np.zeros(c.timesteps)

    def __del__(self):
        np.save("data/" + self.linkName + "SensorValues.npy", self.values)

    def Get_Value(self,t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    