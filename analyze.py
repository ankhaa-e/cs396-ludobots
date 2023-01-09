import numpy as np
import matplotlib.pyplot as plt

backLegSensorValues = np.load("data/b'Torso_BackLeg'MotorValues.npy")
frontLegSensorValues = np.load("data/b'Torso_FrontLeg'MotorValues.npy")
plt.plot(backLegSensorValues,label="Back Leg", linewidth=5)
plt.plot(frontLegSensorValues, label="Front Leg",linewidth=1)
plt.legend()
plt.show()