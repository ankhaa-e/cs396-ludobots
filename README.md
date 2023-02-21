# cs396-ludobots - Assignment 7

This repo is for the Northwestern class COMP_SCI 396: Artificial Life and is based of the Ludobots online course:
https://www.reddit.com/r/ludobots  
https://www.mccormick.northwestern.edu/mechanical/documents/courses/me-495-artificial-life.pdf  

Video: https://youtu.be/a_1bJPQeHPE

To see results:  
run "py button.py"

This will create a 10 bodies and brains, and breifly showcase each of them.
The body shapes are limited to rectangular prisms of varying lengths, widths, and heights.
The diagram below showcases how the random creatures are created.
Essentially, starting from a base body segment, there can be random parts added forwards or backwards, with a chance for a small rotation in x or y axes, and even smaller chance for a smaller roation along the z axis.
Each segment has a chance to grow legs to sides. Each leg then also has a chance to grow a second segment, which is rotated along the x-axis. The legs move along the 1 1 0 axis.
Every sensor is connected to every joint, with random weights. 

![Screenshot](diagram.png)



