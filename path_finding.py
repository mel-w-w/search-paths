'''
Name: Melissa Marie Wang
UNI: msw2178
AI HW1
'''

from pathlib import Path
from queue import PriorityQueue
from typing import Set, Tuple, List

import numpy as np
import numpy.typing as npt

from hw1.utils import neighbors, plot_GVD, PathPlanMode, distance


def cell_to_GVD_gradient_ascent(
    grid: npt.ArrayLike, GVD: Set[Tuple[int, int]], cell: Tuple[int, int]
) -> List[Tuple[int, int]]:
    """
    ~~~~~~~~Step 2~~~~~~~~
    """
    path = [cell]
    while(path[-1] not in GVD):
        # 1-Look at the neighbors of the last cell in the path list
        last_cell = path[-1]
        neighbor_nodes = neighbors(grid, last_cell[0], last_cell[1])

        # 2-Append the successor neighbor with the max value to the path list
        path.append(max(neighbor_nodes))

    # 3-The while loop will end as soon as the last cell in path can be
    # found in the GVD set. 

    # 4-Return path
    return path


def cell_to_GVD_a_star(
    grid: npt.ArrayLike, GVD: Set[Tuple[int, int]], cell: Tuple[int, int], 
    goal: Tuple[int, int]
) -> List[Tuple[int, int]]:

    """
    ~~~~~~Step 3~~~~~~
    """

    frontier = PriorityQueue()
    frontier.put((0, cell))
    frontier_size = [0]
    reached = {cell: {"cost": 0, "parent": None}}
    pointers = {}
    pointers.update({cell:None})
    path = [goal]

    while not frontier.empty():

        # 1-Update frontier_size.
        frontier_size.append(frontier.qsize())

        # 2-Use get() to pop the node in frontier
        # with the cheapest cost
        popped_tuple = frontier.get()
        popped_node = popped_tuple[1]

        if(popped_node == goal):
            frontier.empty()

        # 4-Expand the popped_node to find its children.
        children_nodes = neighbors(grid, popped_node[0], popped_node[1])

       # 5-Add the children to pointers and reached
        for child in children_nodes:
            if(child in reached and g >= reached[child]["cost"]):
                continue
            else:
                if(child not in pointers):
                    pointers.update({child:popped_node})
                    reached.update({child:{"cost": distance([cell], [child]), "parent": pointers[child]}})
                g = distance([cell], reached[child]["parent"]) + distance(reached[child]["parent"],[child])
                h = distance([child], [goal])
                f = g + h
                frontier.put((f,child))
                    
    # TODO: implement this to use the reached table (back pointers) to find
    # the path once you have reached a cell on the GVD.

    while(path[-1] != cell):
        if(path[-1] not in pointers):
            break
        path.append(reached[path[-1]]["parent"])
    path.reverse()

    return path, reached, frontier_size


def GVD_path(
    grid: npt.ArrayLike,
    GVD: Set[Tuple[int, int]],
    A: Tuple[int, int],
    B: Tuple[int, int],
    mode: PathPlanMode
) -> List[Tuple[int, int]]:

    '''
   ~~~~~~~~Step 1~~~~~~~~
    '''

    GVD = set(GVD)
    frontier = [A]
    pointers = {A:None}
    frontier_size = [0]
    path = []
    
    while len(frontier) > 0:

        if(mode == PathPlanMode.BFS):
            # Early Goal Test for A
            if(A==B):
                frontier.clear() # Break the while loop
            else:
                # 1-Update frontier size
                frontier_size.append(len(frontier))

                # 2-Pop the node first added to the frontier
                popped_node = frontier.pop(0)
                
                # 4-Expand the popped node to find its children
                children_nodes = neighbors(grid, popped_node[0], popped_node[1])

                # 5-If the child is in GVD, add to frontier n update pointers
                for child in children_nodes:
                    if(child not in pointers and child in GVD):
                        pointers.update({child:popped_node})
                        frontier.append(child)
               
                # 6-Early Goal Test on all added to frontier
                for node in frontier:
                    if(node == B):
                        frontier.clear()

        elif(mode == PathPlanMode.DFS):
            # Early Goal Test for A
            if(A==B):
                frontier.clear() # Break the while loop
            else:
                # 1-Update frontier size
                frontier_size.append(len(frontier))

                # 2-Pop the node last added to the frontier
                popped_node = frontier.pop()
                
                # 4-Expand the popped node to find its children
                children_nodes = neighbors(grid, popped_node[0], popped_node[1])

                # 5-If the child is in GVD, add to frontier n update pointers
                for child in children_nodes:
                    if(child in GVD):
                        if(child not in pointers):
                            frontier.append(child)
                            pointers.update({child:popped_node})
                            
                # 6-Early Goal Test on all added to frontier
                for node in frontier:
                    if(node == B):
                        frontier.clear()


    # 7-Make the path!
    path.append(B)
    while(path[-1] != A): # while loop will end when we reach the starting cell
        if(path[-1] not in pointers):
            break
        path.append(pointers[path[-1]])
        
    path.reverse() # reverse path before you return it!

    return path, pointers, frontier_size

def compute_path(
    grid,
    GVD: set[tuple],
    start: tuple,
    goal: tuple,
    outmode: PathPlanMode = PathPlanMode.GRAD,
    inmode: PathPlanMode = PathPlanMode.DFS):

    """ Compute the path on the grid from start to goal using the methods
    implemented in this file. 
    Returns:
        list: a list of tuples represent the planned path. 
    """

    if outmode == PathPlanMode.GRAD:
        start_path = cell_to_GVD_gradient_ascent(grid, GVD, start)
        end_path = list(reversed(cell_to_GVD_gradient_ascent(grid, GVD, goal)))
    else:
        start_path = cell_to_GVD_a_star(grid, GVD, start, goal)[0]
        end_path = list(reversed(cell_to_GVD_a_star(grid, GVD, goal, start)[0]))
    mid_path, reached, frontier_size = GVD_path(
        grid, GVD, start_path[-1], end_path[0], inmode)
    return start_path + mid_path[1:-1] + end_path


def test_world(
    world_id, 
    start, 
    goal,
    outmode: PathPlanMode = PathPlanMode.GRAD,
    inmode: PathPlanMode = PathPlanMode.DFS,
    world_dir="worlds"):

    print(f"Testing world {world_id} with modes {inmode} and {outmode}")
    grid = np.load(f"{world_dir}/world_{world_id}.npy")
    GVD = set([tuple(cell) for cell in np.load(
        f"{world_dir}/world_{world_id}_gvd.npy")])
    path = compute_path(grid, GVD, start, goal, outmode=outmode, inmode=inmode)
    print(f"Path length: {len(path)} steps")
    plot_GVD(grid, world_id, GVD, path)
