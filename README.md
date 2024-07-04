# search-paths

This project was completed for the course Artificial Intelligence in Columbia University.

The goal of the project was to explore how grid worlds, especially in robotics, could be posed as search problems.

For the sake of academic privacy, I have only included the code I've written for the project, `path_finding.py` and not what my TAs or the Professor prepared for the project.

The functions I designed and implemented are listed below, along with their descriptions:
- **GVD path(grid, GVD, A, B, mode)**: takes in a grid and Generalized Voronoi Diagram (GVD), as well as start and goal locations A and B (both of which are tuples of cell indices in the GVD). This method runs both a depth-first and breadth-first search, depending on the mode paramter.
- **cell_to_GVD_gradient_ascent()**: takes in the grid, GVD, and a given starting cell, and returns a path (list of adjacent cells). The function performs a local gradient ascent search on a grid to find the path from the starting cell to the nearest cell in the GVD.
- **cell_to_GVD_a_star(...)**: performs A* search to find the optimal path from starting cell A to goal cell B, ensuring that both cells eventually reach a connection point on the GVD.
