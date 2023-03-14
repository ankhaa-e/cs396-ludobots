import os
from solution import SOLUTION
import time

s = SOLUTION(1)
s.Write_Files()
s.Start_Simulation(True)
s.Wait_For_Simulation_To_End()

