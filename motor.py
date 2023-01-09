import constants as c
import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim

class MOTOR:
    def __init__(self,jointName,robotId, amplitude, frequency, phaseOffset):
        self.jointName = jointName
        self.robot = robotId
        self.amplitude = amplitude
        self.frequency = frequency
        self.phaseOffset = phaseOffset
        self.values = np.zeros(c.timesteps)
        
    def __del__(self):
        pass #np.save("data/" + str(self.jointName) + "MotorValues.npy", self.values)

    def Set_Value(self, desiredAngle):

        pyrosim.Set_Motor_For_Joint(
            bodyIndex = self.robot,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = desiredAngle,
            maxForce = c.force)