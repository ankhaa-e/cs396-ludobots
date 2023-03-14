import os
from parallelHillClimber import PARALLEL_HILLCLIMBER
import random

random.seed(121212134524524)

phc = PARALLEL_HILLCLIMBER()
phc.Evolve()
phc.Show_Best()


