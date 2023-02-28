import os
from solution import SOLUTION
import time

s = SOLUTION(1)
s.Write_Files()
s.Start_Simulation(True)
s.Wait_For_Simulation_To_End()

#for _ in range(10):
#    os.system("py generate.py")
#    os.system("py simulate.py GUI x")
