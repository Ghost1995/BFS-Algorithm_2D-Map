# BFS-Algorithm_2D-Map

**Problem Statement**

Use Breadth First Search to find optimal path between any given initial point and goal point. Consider workspace as a 8-connected space. Use Half-plane and semi-algebraic models to represent obstacle space. Show optimal path using a simple graphical interface.

**Running the Code**

To get the output from the function, the minimum command that need to be given is:
path = bfs(1,[],[]);

**Description**

The source code contains 2 functions. These functions have been described below.

The first function is bfs.m. This function takes as input the resolution for the grid, start position and the goal position. If no resolution is defined, it returns an error. If no start or goal position is defined, it gives the user a chance to select the start and goal position in the grid (figure). After getting all the inputs, the code applies breadth first search to identify the optimal path for the given inputs. The output is a figure showing the start node, goal node, all the nodes which were expanded, and the path. It also gives path as matrix as an output.

The second function is collide.m. As the name suggests, this function checks whether a point collides with an obstacle or not. It also checks if the point lies inside the workspace or not. In case, the point to be checked is not a grid point, it returns the corresponding grid point as the second output but only if it lies in the free space of the workspace.

