from simulation import SIMULATION
import sys

directOrGUI = sys.argv[1]
solutionId = sys.argv[2]
rem = sys.argv[-1]
simulation = SIMULATION(directOrGUI, solutionId, rem == solutionId)
simulation.Run()
simulation.Get_Fitness()