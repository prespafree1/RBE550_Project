#Abedin Sherifi
#RBE550
#Project AUT Group 1

import heapq
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
import mpl_toolkits.mplot3d as plt3d
import time
import os

#A* Algorithm node class
class Node:
    def __init__(self, parent_node=None, posn=None, diff=None):
        self.parent_node = parent_node
        self.posn = posn
        self.diff = diff

        self.g = 0
        self.h = 0
        self.f

    @property
    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.posn == other.posn

    def __hash__(self):
        return hash(self.posn)

#A path is returned in form of list of tuples
def A_Star(occupancy_grid, start, goal):
    
    #Heuristic cost
    #https://stackoverflow.com/questions/53116475/calculating-diagonal-#distance-in-3-dimensions-for-a-path-finding-heuristic
    def heuristic(current):
        dx = abs(current.posn[0] - goal_node.posn[0])
        dy = abs(current.posn[1] - goal_node.posn[1])
        dz = abs(current.posn[2] - goal_node.posn[2])
        dmin = min(dx, dy, dz)
        dmax = max(dx, dy, dz)
        dmid = dx + dy + dz - dmin - dmax
        return (
            (1.7 - 1.4) * dmin
            + (1.4 - 1) * dmid
            + dmax * 1.4
        )

    def diff(posn):
        sum = abs(posn[0]) + abs(posn[1]) + abs(posn[2])
        if sum == 3:
            return 1.7
        elif sum == 2:
            return 1.4
        else:
            return 1

    #Open and Closed List init
    Open_List = []
    Closed_List = set()
    heapq.heapify(Open_List)

    #Setting Start and Goal nodes
    start_node = Node(None, start)
    goal_node = Node(None, goal)

    #Add start node to open list
    heapq.heappush(Open_List, start_node)

    #While loop till goal reached
    while len(Open_List) > 0:

        #Retreive current node from open list
        current_node = heapq.heappop(Open_List)
        Closed_List.add(current_node)

        #Give path if goal found by going back from goal point to start point
        if current_node == goal_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.posn)
                current = current.parent_node
            return path[::-1]

        #All directions that a new node can be choosen from
        new_nodes = []
        for new_posn in [
            (0, -1, 0),
            (0, 1, 0),
            (-1, 0, 0),
            (1, 0, 0),
            (0, 0, 1),
            (0, 0, -1),
            (-1, -1, 0),
            (-1, 1, 0),
            (1, -1, 0),
            (1, 1, 0),
            (0, -1, -1),
            (0, 1, -1),
            (0, -1, 1),
            (0, 1, 1),
            (-1, 0, -1),
            (1, 0, -1),
            (-1, 0, 1),
            (1, 0, 1),
            (-1, -1, -1),
            (-1, 1, -1),
            (1, -1, -1),
            (1, 1, -1),
            (-1, -1, 1),
            (-1, 1, 1),
            (1, -1, 1),
            (1, 1, 1),
        ]:

            #Node posn
            node_posn = (
                current_node.posn[0] + new_posn[0],
                current_node.posn[1] + new_posn[1],
                current_node.posn[2] + new_posn[2],
            )

            #Verifying node posn is within range
            if (
                node_posn[0] > (occupancy_grid.shape[2] - 1)
                or node_posn[0] < 0
                or node_posn[1] > (occupancy_grid.shape[1] - 1)
                or node_posn[1] < 0
                or node_posn[2] < 0
                or node_posn[2] > (occupancy_grid.shape[0] - 1)
            ):
                continue

            #Obstacle detection
            if occupancy_grid[node_posn[2], node_posn[1], node_posn[0]] != 0:
                continue

            #Get new node
            new_node = Node(current_node, node_posn, diff(new_posn),)

            #Append new node
            new_nodes.append(new_node)

        #Go through new nodes
        for new_node in new_nodes:

            #New node is on closed list
            for closed_node in Closed_List:
                if new_node == closed_node:
                    continue

            #New node g and h values
            new_node.g = current_node.g + new_node.diff
            new_node.h = heuristic(new_node)
       
            #If new node already in open list
            for open_node in Open_List:
                if new_node == open_node and new_node.g > open_node.g:
                    continue

            #Push new node to open list
            heapq.heappush(Open_List, new_node)


def main():
    occupancy_grid = np.array(
        [
            [
                [0, 1, 0, 1, 0, 0, 0, 0, 1, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
            [
                [0, 1, 0, 1, 0, 0, 0, 0, 1, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
            [
                [0, 1, 0, 1, 0, 0, 0, 0, 1, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
	    [
                [0, 1, 0, 1, 0, 0, 0, 0, 1, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
	    [
                [0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
	    [
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
            [
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
            [
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
            [
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
            [
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ],
        ],
    )

    start = (0, 0, 1)
    goal = (9, 9, 1)

    start_time = time.time()
    path = A_Star(occupancy_grid, start, goal)
    end_time = time.time()
    print("Time to Solution", (end_time - start_time))

    fig = plt.figure()
    fig.canvas.set_window_title('3D A*')
    ax = fig.add_subplot(111, projection="3d")
    print("Occupancy Grid",occupancy_grid.shape)
    for z in range(0, occupancy_grid.shape[0]):
        for y in range(0, occupancy_grid.shape[1]):
            for x in range(0, occupancy_grid.shape[2]):
                #Obstacles denoted by '1'
                if occupancy_grid[[z], [y], [x]] != 0:
                    ax.scatter(x, y, z, s=50,marker="s", c="black")

    for point in path:
        ax.scatter(point[0], point[1], point[2], s=100,marker="o", c="purple")

    ax.scatter(start[0], start[1], start[2], s=150,marker="o", c="red")
    ax.scatter(goal[0], goal[1], goal[2], s=150,marker="o", c="green")
    for i in range(len(path)-1):
        line = plt3d.art3d.Line3D([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]],
                                  [path[i][2], path[i+1][2]], color='green', linewidth=5)
        ax.add_line(line)

    ax.set_title("3D A* Implementation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    print("Number of Waypoints", len(path))
    print("Solution Waypoints", path)
    text_file = open('Waypoints.txt', 'w')
    np.savetxt(text_file, path, fmt='%s')
    text_file.close()
    plt.show()

if __name__ == "__main__":
    main()
