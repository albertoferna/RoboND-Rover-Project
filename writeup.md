---
### Writeup / README

Most of the explanation of what I've done is in the notebook [Rover_Project_Notebook](https://github.com/albertoferna/RoboND-Rover-Project/blob/master/code/Rover_Project_Notebook.ipynb). There is also an 'as run' [version in html](https://github.com/albertoferna/RoboND-Rover-Project/blob/master/code/Rover_Project_Notebook.html)


### Exploration explanation

The only explanation missing is what I tried to do to ensure full map exploration. While the simple thing in a map similar to a labyrinth is to always follow one wall, I kept track of places visited. I used that information to penalize the steering direction towards areas already visited. In the end, it is not very robust. However, with enough time, the rover visits consistently over 70% of the map.

### Picking up rocks

I wrote code to pick up sample rocks. It is just a greedy implementation in which the rover goes for the rock as soon as it locates it. It is not elegant but it manages to pick up a few of them. This behaviour can be turned off by setting Rover.pick_up_samples to False in the class initialization in drive_rover.py

### Running the code

In my machine the code runs at quite different frame rates from time to time. The submitted video was run at between 30-50 FPS (except at the very beginning for some reason) at 800x600.

Something that gave me a lot of problems until I figured out what was wrong was communication with the simulator. Due to my locale (es_ES) commands were either not read, or even read incorrectly. I solved the issue by starting the simulator with a script that changed the locale:
```
#!/bin/bash
export LC_ALL="en_US.UTF-8"
./Roversim.x86_64
```
