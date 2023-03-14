# cs396-ludobots - Final Project (Engineering Route)

This repo is for the Northwestern class COMP_SCI 396: Artificial Life and is based of the Ludobots online course:  
https://www.reddit.com/r/ludobots  
https://www.mccormick.northwestern.edu/mechanical/documents/courses/me-495-artificial-life.pdf    
https://www.thunderheadeng.com/pyrosim

Video: 

## What is this?
![GIF](showcase.gif)
This repo is for the final project for the class COMP_SCI 396: Artificial Life, taught by Professor Sam Kriegman. Creatures are randomly generated, then mutated over multiple generations to become optimized for a specified task. The evolutionary algorithm selects the best children. The end result is a completely random creature specified to do the designated task, with no input from a human. This project was built off of r/ludobots and used pyrosim, both are linked above. Below is a quick showcase of the possible results of this project.

# Methods

## Task
The creature is measured by a fitness function of the distance traveled in the negative x direction.

## Parallel Hill Climbing
Over 500 generations, 10 random bots will be created. After each generation, each robot is mutated. If the child has a better fitness, the child replaces the parent. Otherwise, the original parent is mutated again for the next generation.
The Diagram below showcases the process.

![PHC](diagram-phc.png)

## Design Space
Essentially, starting from a base body segment, there can be random parts added forwards or backwards, with a chance for a small rotation in x or y axes, and even smaller chance for a smaller roation along the z axis.
Each segment has a chance to grow legs to sides. Each leg then also has a chance to grow a second segment, which is rotated along the x-axis. The legs move along the 1 1 0 axis.
Every sensor is connected to every joint, with random weights. 
The body parts are limited to rectangular prisms of varying lengths, widths, and heights.
The body is stored in a series of body segment classes that have children dictating connections from the base starting point.
The diagram below showcases how the random creatures are created.
![Morphology](diagram.png)

## Brain
The brain is connected of randomly chosen sensors, each part has a 1/3 chance to be a sensor when generated. Ends of limbs have a 100% chance to be a sensor. Every sensor is connected to every motor. Body parts that are sensors are colored green, and it is guarenteed to be at least one sensor. The diagram showcases a possible example.
![Brain](diagram-brain)

## Mutations
After being generated, the creature can go several possible mutations with a random chance. The diagram below showcases the possibilities. There is a 1/4 chance the body will be extended, with the same probabilities for size and for having limbs as if it were being generated. There is a 1/4 chance that a body segment will be deleted from one of the ends, including any limbs it had. There is a 1/4 chance of one of the ends of the creature to gain limbs if it didn't have any or lose the limbs if it does. There is a 1/4 chance that no body segments are added or deleted. There is a 100% chance that one of the weights of sensor-joint synapses to be changed to another random value.

The diagram belows the possibilities for which the body can be mutated.
![Mutations](diagram-mutate.png)

The diagram below gives examples of how changes can be made to a body.
![Mutations](diagram-mutationsEX.png)



## Results

Video Link: https://youtu.be/_XtVIE-YPco

This video summarizes and showcases the findings of this project. Below is a graph of fitness of 10 instances of Parallel Hill Climbers, with a population size of 10 and evolved over 500 generations for a total of 50,000 simulations. Each robot was given 1000 timesteps to travel as far as possible in the negative x direction. 

![FitnessPlot](fitnessPlot.png)

In earlier iterations, creatures would evolve into 2 main forms, very long snakes, that would move forward by moving in a wavelike method, or 2 segmented creatures with no limbs that would inch forward. To make things more interesting, I forced adding new segments when the body has too few joints and removing segments when there are too many. 
This resulted in the evolved creatures falling into 3 categories, two segmented with limbs,long snakes with a few limbs that was similar to the original snake, or short 3-4 segmented creatures with no limbs that could jump forward. The highest preforming were the legless hoppers, which were also admitedlly also the least interesting.


### Limitations and Room for Improvement

One of the issues with the way creatures evolve is that it is pretty easy to fall into a rut and evolve further. If there are multiple mutations that need to take place in order to improve, it is impossible to climb out as the mutations are only saved if it immediately improved. A possible way to explore mutations is instead of parallel hillclimbing from 10 (or more) different starting points, start with one robot, mutate 10 different ways, and after 5 generations replace the robot with the one that performed the best. Afterwards, we can evolve the best robot in 10 different ways and repeat every 5 generations. This way, mutations are allowed to compound on each other to further explore possible configurations. In other terms, instead of being in a hole and only being able to take small steps, it can take larger steps to find an optimal path. 


## How To Run

To parallel hill climb:  
"py search.py"

To view 10 best results from 50,000 simulations:  
"py showcase.py"  
(Note that simulations after the first may not pop into top of desktop and may be buried by other windows open.)

To view a random, untrained creature:  
"py button.py"

To view a creature evolve for a 500 generations, showcasing mutations that evolve it:  
"py hillclimb.py"

To view fitness graphs:
"py analyze.py"