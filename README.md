# cs396-ludobots - Assignment 5

This repo is for the Northwestern class COMP_SCI 396: Artificial Life and is based of the Ludobots online course:
https://www.reddit.com/r/ludobots  
https://www.mccormick.northwestern.edu/mechanical/documents/courses/me-495-artificial-life.pdf  

Video: https://youtu.be/SURhGh6jzms  

To see results:  
run "py button.py"

This will create a random untrained brain and then run a brain that had the best results from the search funtion.  
To train a new brain, run "py search.py"  
Note that his will delete the trained brain.  

The fitness function is the same as the quadraped, a greater fitness is a lower x position value.  

The design was based on an ant, with 3 body segments and 6 legs. Originally the body had different sizes, but that would cause it to flip over. Furthermore, the legs are out to the side, so to allow forward motion, I added axes to the legs, 1 0 1 for the right legs and 1 0 -1 for the left legs. I also added a parameter that allowed the legs to be set at an angle, so the legs are put out at 45 degrees. However, with the middle legs they would often cause an imbalance of the creature that made it difficult to get a realistic gait.  
