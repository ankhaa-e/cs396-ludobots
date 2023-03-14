from solution import SOLUTION
import constants as c
import copy

solution = SOLUTION(0)
best = {}

gen = 0
fitnesses = {}

solution.Start_Simulation(True)
solution.Wait_For_Simulation_To_End()

bestFitness = solution.fitness
best.update({gen:solution})
fitnesses.update({gen:bestFitness})
print("Gen: ", gen," Fitness: ", solution.fitness)
gen += 1
while gen < c.numGenerations:

    child = copy.deepcopy(solution)
    child.Set_ID(gen)
    if len(child.joints) < 4:
        child.Mutate(.1)
    elif len(child.joints) > 12:
        child.Mutate(.4)
    else:
        child.Mutate()

    child.Start_Simulation()
    child.Wait_For_Simulation_To_End()
    if child.fitness < bestFitness:
        bestFitness = child.fitness
        best.update({gen:solution})
        fitnesses.update({gen:bestFitness})

        solution.Write_Files()
        solution.Write_Body()

        solution.Start_Simulation(True)
        print("Gen: ", gen," Fitness: ", solution.fitness)
    gen+=1

print(fitnesses)