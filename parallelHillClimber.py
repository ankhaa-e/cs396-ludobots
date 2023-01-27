from solution import SOLUTION
import constants as c
import copy
import random
import os

import constants as c
class PARALLEL_HILLCLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.nextAvailableId = 0
        self.parents = {} # SOLUTION()
        self.children = {}
        for i in range(c.populationSize):
            self.parents.update({i:SOLUTION(i)})
            self.nextAvailableId += 1
        self.currentGen = 0

    def Evolve(self):
        self.Evaluate(self.parents)
        while self.currentGen < c.numGenerations:
            self.Evolve_For_One_Generation()
            self.currentGen+=1

    def Evolve_For_One_Generation(self):
        for key, parent in self.parents.items():
            self.Spawn(key, parent)
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Evaluate(self, solutions):
        for solutuion in solutions.values():
            solutuion.Start_Simulation()
        for solutuion in solutions.values():
            solutuion.Wait_For_Simulation_To_End()

    def Spawn(self,key, parent):
        self.children[key] = copy.deepcopy(parent)
        self.children[key].Set_ID(self.nextAvailableId)
        self.nextAvailableId += 1

    def Mutate(self):
        for child in self.children.values():
            i = random.randint(0,c.numSensorNeurons-1)
            j = random.randint(0,c.numMotorNeurons-1)
            child.weights[i][j] = random.random() * 2 - 1

    def Print(self):
        print("\nGeneration ", self.currentGen)
        for (i, parent), (j,child) in zip(self.parents.items(),self.children.items()):
            print("Parent ",i," Fitness: ",parent.fitness, " Child " ,j, " Fitness: ", child.fitness)
        print("")
    
    def Select(self):
        for (i, parent), (j,child) in zip(self.parents.items(),self.children.items()):
            if parent.fitness > child.fitness:
                self.parents[i] = self.children[j]
    
    def Show_Best(self):
        bestFitness = float("inf")
        bestSolution = None
        for solution in self.parents.values():
            if solution.fitness < bestFitness:
                bestFitness = solution.fitness
                bestSolution = solution
        print("Best Fitness: ",bestSolution.fitness, " From Brain ", bestSolution.id)
        bestSolution.Start_Simulation(True)
        bestSolution.Wait_For_Simulation_To_End()
        bestSolution.Generate_Brain()

    

