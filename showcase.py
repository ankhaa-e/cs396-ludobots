from simulation import SIMULATION
from solution import SOLUTION
import pybullet as p
import time


for i in range(10):
    simulation = SIMULATION("GUI", str(i+1), False, True)
    simulation.Run()
    del(simulation)
    time.sleep(1)
