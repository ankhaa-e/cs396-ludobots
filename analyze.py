import numpy as np
import matplotlib.pyplot as plt

fitness1 = np.load("data/fitness1.npy")
fitness2 = np.load("data/fitness2.npy")
fitness3 = np.load("data/fitness3.npy")
fitness4 = np.load("data/fitness4.npy")
fitness5 = np.load("data/fitness5.npy")
plt.plot(fitness1, label="Trial 1",linewidth=1)
plt.plot(fitness2, label="Trial 2",linewidth=1)
plt.plot(fitness3, label="Trial 3",linewidth=1)
plt.plot(fitness4, label="Trial 4",linewidth=1)
plt.plot(fitness5, label="Trial 5",linewidth=1)
plt.title("Fitness over Generations")
plt.xlabel("Generation")
plt.ylabel("abs(fitness)")
plt.legend()
plt.show()
#plt.savefig('fitnessPlot.png')