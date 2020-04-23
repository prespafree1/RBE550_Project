# Derek Larson
# Final Project
# 4/15/2020
# Depth first search.


import numpy
import os
import math
import time

'''
Define global_variables
'''
x_blocks = 10           # Number of city blocks in x-dimension
y_blocks = 10           # Number of city blocks in y-dimension
min_block_height = 5    # Min height of a block (m)
max_block_height = 200  # Max height of a block (m)

block_w = 50            # Depth of each block (m)
block_d = 50            # Width of each block (m)
street_w = 20           # Width of streets (m)

cell_res = 10           # Cell resolution for 3D-matrix (m)

def make_3d_mat(file_name):
    """
    Generate 3D-matrix given 2D height map of block heights
    :param file_name: 2D height map .txt file where each value is height of given block
    :return: mat
    """

    ################# Convert input file to 2D matrix of block heights
    height_mat = []

    try:
        f_check = open(file_name, "r")

        for line in f_check:
            row = [int(i) for i in line.split(",")]
            height_mat.append(row)

        f_check.close()

    except IOError:
        print('Error: file name provided to make_3d_mat does not exist')
        mat = []
        return mat

    ################# build 3D matrix
    mat = []
    mat_row = []
    mat_level = []

    # Determine number of cells in each dimension
    x_dim = int(round((x_blocks*block_w + (x_blocks + 1)*street_w)/cell_res))
    y_dim = int(round((y_blocks*block_d + (y_blocks + 1)*street_w)/cell_res))
    z_dim = int(round(2*max_block_height/cell_res))

    street_cells = int(round(street_w / cell_res))
    x_block_and_str = int(round((street_w + block_w)/cell_res))
    y_block_and_str = int(round((street_w + block_d)/cell_res))
    x_block = int(round(block_w/cell_res))
    y_block = int(round(block_d/cell_res))

    for k in range(z_dim):
        for j in range(y_dim):
            for i in range(x_dim):

                # Check if within a block (set to 1) or free (set to 0)
                if i <= street_cells - 1 or j <= street_cells - 1:
                    mat_row.append(0)
                elif (i - street_cells) % x_block_and_str > x_block - 1:
                    mat_row.append(0)
                elif (j - street_cells) % y_block_and_str > y_block - 1:
                    mat_row.append(0)
                else:
                    x_block_num = int(round(math.floor(i/(street_w + block_w)/cell_res)))
                    y_block_num = int(round(math.floor(j/(street_w + block_d)/cell_res)))
                    if k * cell_res > height_mat[y_block_num][x_block_num]:
                        mat_row.append(0)
                    else:
                        mat_row.append(1)

            mat_level.append(mat_row)
            mat_row = []
        mat.append(mat_level)
        mat_level = []

    return mat


class Node(object):
    def __init__(self, x, y, z, phi, parent, p_edge, cost):
        self.x = x              # X coordinate of node
        self.y = y              # Y coordinate of node
        self.z = z              # Z coordinate of node
        self.phi = phi          # heading of node
        self.parent = parent    # parent node
        self.p_edge = p_edge    # edge to get to node from parent [x,y,z]
        self.cost = cost        # Past cost to node
    def __repr__(self):
        nString = "Node({0}, {1}, {2}, {3})"
        return nString.format(self.x, self.y, self.z, self.phi)

# The function creates a shell script of waypoints given an array of points (x, y, z).
def create_waypoint_file(waypoints):
    # delete waypoint_script if exists, since going to append to it
    if os.path.exists("waypoint_script.sh"):
        os.remove("waypoint_script.sh")
    # create waypoint_script.sh, and append waypoints.
    f = open('waypoint_script.sh', 'a')
    for i in waypoints:
        numpy.savetxt(f, ["rosrun rotors_gazebo waypoint_publisher " + str(i.x * 10) +
                          " " + str(i.y * 10) + " " + str(i.z * 10) + " " + str(i.phi) + " 0 __ns:=firefly"], fmt='%s')
    f.close()
    os.chmod("waypoint_script.sh", 0o777)

    return

# This functions checks to see if the node passed in is already in the list of visited nodes
def node_already_visited(node, visited_nodes):
    for w in visited_nodes:
        if (w.x == node.x and w.y == node.y and w.z == node.z):
            return True
    return False

# This function performs the depth first search on the maze passed in and finals a path if one exisits.
def depthSearch(maze, num_cols, num_rows):
    print "Starting depth search"
    solved = False
    num_of_moves = 0
    temp_path = []
    path = []
    visited = []
    start = Node(0, 0, 1, 0,[],[],cost=0)
    end = Node(64, 64, 1, 0,[],[],cost=0)
    temp_path.append(start)
    visited.append(start)
    # Start timer and solve maze
    start_time = time.time()

    while temp_path:
        current_node = temp_path.pop()
        path.append(current_node)
        num_of_moves += 1
        # Check to see if popped node is the goal, if so end search
        if (current_node.x == end.x and current_node.y == end.y and current_node.z == end.z):
            solved = True
            print "SOLVED"
            end_time = time.time()
            solve_time = end_time - start_time
            print("Maze solved in {} moves in {} seconds".format(num_of_moves, solve_time))
            break
        # check below Y direction Cell, push if not visited or object
        below_y_node = Node(current_node.x, current_node.y + 1, current_node.z, 0,[],[],cost=0)
        if ((below_y_node.y < num_rows - 1) and ((maze[below_y_node.z][below_y_node.y][below_y_node.x]) == 0)\
                and not node_already_visited(below_y_node, visited)):
            temp_path.append(below_y_node)
            visited.append(below_y_node)
        # check above Y Cell, push if not visited or object
        above_y_node = Node(current_node.x, current_node.y - 1, current_node.z, 0,[],[],cost=0)
        if ((above_y_node.y > 0) and ((maze[above_y_node.z][above_y_node.y][above_y_node.x]) == 0) \
                and not node_already_visited(above_y_node, visited)):
            temp_path.append(above_y_node)
            visited.append(above_y_node)
        # check right Cell, push if not visited or object
        right_node = Node(current_node.x + 1, current_node.y, current_node.z, 0,[],[],cost=0)
        if ((right_node.x < num_cols) and ((maze[right_node.z][right_node.y][right_node.x]) == 0) \
                and not node_already_visited(right_node, visited)):
            temp_path.append(right_node)
            visited.append(right_node)
        # check left Cell, push if not visited or object
        left_node = Node(current_node.x - 1, current_node.y, current_node.z, 0,[],[],cost=0)
        if ((left_node.x > 0) and ((maze[left_node.z][left_node.y][left_node.x]) == 0) \
                and not node_already_visited(left_node, visited)):
            temp_path.append(left_node)
            visited.append(left_node)
        # check above Z Cell, push if not visited or object
        above_z_node = Node(current_node.x, current_node.y, current_node.z + 1, 0,[],[],cost=0)
        if ((above_z_node.z < 11) and ((maze[above_z_node.z][above_z_node.y][above_z_node.x]) == 0) \
                and not node_already_visited(above_z_node, visited)):
            temp_path.append(above_z_node)
            visited.append(above_z_node)
        # check below Z Cell, push if not visited or object
        below_z_node = Node(current_node.x, current_node.y, current_node.z - 1, 0,[],[],cost=0)
        if ((below_z_node.z > 0) and ((maze[below_z_node.z][below_z_node.y][below_z_node.x]) == 0) \
                and not node_already_visited(below_z_node, visited)):
            temp_path.append(below_z_node)
            visited.append(below_z_node)
    if not solved:
        print("no path found")
    return path

# Main functions that creates the city map from the text file and then solves it.
if __name__ == "__main__":

    num_cols = 72
    num_rows = 72
    print("Create map")
    matrix = make_3d_mat("height_map2.txt")
    final_path = depthSearch(matrix, num_cols, num_rows)
    create_waypoint_file(final_path)
