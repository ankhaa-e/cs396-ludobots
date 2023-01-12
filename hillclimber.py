from solution import SOLUTION
import copy
import random
import os

import constants as c
class HILLCLIMBER:
    def __init__(self):
        self.parent = SOLUTION()
        self.currentGen = 0

    def Evolve(self):
        self.parent.Evaluate()
        while self.currentGen < c.numGenerations:
            self.Evolve_For_One_Generation()
            self.currentGen+=1
        self.Show_Best()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate()
        self.Print()
        self.Select()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        i = random.randint(0,2)
        j = random.randint(0,1)
        self.child.weights[i][j] = random.random() * 2 - 1

    def Print(self):
        print("Parent Fitness: ",self.parent.fitness, " Child Fitness: ", self.child.fitness)
    
    def Select(self):
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child
    
    def Show_Best(self):
        self.parent.Generate_Brain()
        os.system("py simulate.py GUI")

    

