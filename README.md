# cs396-ludobots - Assignment 5

To see results:
run "py button.py"

This will create a random untrained brain and then run a brain that had the best results from 100 generations of training.

The fitness function is the same as the quadraped.

The design was based on an ant, with 3 body segments and 6 legs. Originally the body had different sizes, but that would cause it to tip over. Furthermore, the legs are out to the side, so to allow forward motion, I added axes to the legs, 1 0 1 for the front legs and 1 0 -1 for the back legs.

However, the best result is consistently from the "ant" turning 90 degrees and walking that way at an angle. 

