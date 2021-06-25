# Pathfinding

This project shows pathfinding process based on grid.


## Features

**In the F2 project, user can set heuristics, maps, weight through UI settings.**

![F2](/GIF/F2.gif)

### Heuristics
1. Euclidean: sqrt(xDiff^2 + yDiff^2)
2. Octile: min(xDiff,yDiff) * sqrt(2) + max(xDiff,yDiff) - min(xDiff,yDiff)
3. Chevyshev: max(xDiff,yDiff)
4. Manhattan: xDiff + yDiff

### Smoothing

Used Catmull-Rom algorithm to smooth the nodes.

### Rubberbanding

Find all the middle nodes that does not affect the path.

**In the F3 project, user can check various analyzations used in grid terrain.**

![F3](/GIF/F3.gif)

### Analyzed layers 

1. Openness: Proximity to static obstacles
2. Visibility: How many squares are visible
3. Occupancy map: Probability of where the player is. Implemented with propagation algorithm.

### Search 

Shows where agent have searched. Fades by time.

### Hide and seek

Red agent starts to look for the blu agent, showing its sight.
Once the red agent finds the blue agent, it generates the propagation spot based on where the blue agent was last seen


## Download

You can download the executable file [here](https://github.com/Picbridge/Pathfinding/raw/main/Executable.zip)

**The project consists of 2 types of smaller projects. This can be switched by pressing F2, and F3.**

All the controls are made with left mouse click. (Red agent can be controlled via right click in project F3)
