"""
RBE550, Project Group 1

@author: Jesse Morzel
"""

"""
Define imports
"""
import random as rand
import math
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib import interactive
import time

'''
Define global_variables
'''
# Map definition variables

x_blocks = 10               # Number of city blocks in x-dimension
y_blocks = 10               # Number of city blocks in y-dimension
min_block_height = 5        # Min height of a block (m)
max_block_height = 200      # Max height of a block (m)

block_w = 50                # Depth of each block (m)
block_d = 50                # Width of each block (m)
street_w = 20               # Width of streets (m)

cell_res = 10               # Cell resolution for 3D-matrix (m)

h_map_file = "height_map2.txt"   # City block height map .txt file to use

# Maximum navigation limits for FireFly
x_max = x_blocks*block_w + (x_blocks + 1)*street_w
y_max = y_blocks*block_d + (y_blocks + 1)*street_w
z_max = max_block_height/2

# RRT Variables
trans_lim = 10
phi_lim = math.pi/2

dx_tol = 0.5                # Goal tolerance (x)
dy_tol = 0.5                # Goal tolerance (y)
dz_tol = 0.5                # Goal tolerance (z)
dphi_tol = 0.2              # Goal tolerance (phi)

goal_weight = 0.075         # Probability of choosing goal node
max_tree_size = 10000        # Max tree size


def make_height_map(num_x_blocks, num_y_blocks, file_name):
    """
    Make a x-by-y 2D matrix where cell value corresponds to height of block
    :param num_x_blocks:
    :param num_y_blocks:
    :param file_name: .txt file to create
    :return: mat: list of lists where value corresponds to height (m)
    """

    # Init variables
    mat_row = []
    mat = []
    max_block_dist = math.sqrt(2)*max(num_x_blocks,num_y_blocks)/2

    # Write to .txt file. If it already exists, display error
    try:
        f_check = open(file_name, "r")
        f_check.close()
        print('Error: height_map txt file already exists. Change filename')
        return mat
    except IOError:
        f_new = open(file_name, "w+")

        for i in range(num_y_blocks):
            for j in range(num_x_blocks):
                # Determine cell height
                block_dist = math.sqrt((j - num_x_blocks/2)**2 + (i - num_y_blocks/2)**2)
                h = int(round(min_block_height + rand.random()*(max_block_height - min_block_height)*abs(1- block_dist/max_block_dist)))
                mat_row.append(h)

                # Write to .txt file
                if j < (num_x_blocks - 1):
                    f_new.write(str(h)+", ")
                else:
                    f_new.write(str(h) + " \n")

            mat.append(mat_row)
            mat_row = []

        f_new.close()

    return mat

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

def make_obs_list(file_name):
    """
    Create list of obstacles given input height_map of obstacle heights
    :param obs_list: list of obstacles where each obstacle defined as [p1, p2, p3, p4] where p = [x,y,z]
    :return: obs_list
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
        return []

    # Init obs list
    obs_list = []

    for j in range(len(height_mat)):
        for i in range(len(height_mat[0])):

            # Define corner points of cube
            p1 = [street_w + i*(block_w + street_w), street_w + j*(block_d + street_w), 0]
            p2 = [(i+1) * (block_w + street_w), p1[1], 0]
            p3 = [p1[0], (j+1)*(block_d + street_w), 0]
            p4 = [p1[0], p1[1], height_mat[j][i]]

            obs_list.append([p1, p2, p3, p4])

    return obs_list

def make_obs_models(obs_list, file_name):
    """
    Create .txt file containing obstacle models defined by obs_list, in Gazebo model format
    :param obs_list: list of obstacles
    :param file_name: output .txt file
    :return:
    """

    # Write to .txt file. If it already exists, display error
    try:
        f_check = open(file_name, "r")
        f_check.close()
        print('Error: make_obs_models txt file already exists. Change filename')
    except IOError:
        f_new = open(file_name, "w+")

        # Add model def within main section of world
        for i in range(len(obs_list)):
            f_new.write("    <model name='unit_box_%03i'>\n" % i)
            f_new.write("      <pose frame=''>0 0 0 0 -0 0</pose>\n")
            f_new.write("      <static>1</static>\n")
            f_new.write("      <link name='link'>\n")
            f_new.write("        <inertial>\n")
            f_new.write("          <mass>5000</mass>\n")
            f_new.write("          <inertia>\n")
            f_new.write("            <ixx>200</ixx>\n")
            f_new.write("            <ixy>0</ixy>\n")
            f_new.write("            <ixz>0</ixz>\n")
            f_new.write("            <iyy>200</iyy>\n")
            f_new.write("            <iyz>0</iyz>\n")
            f_new.write("            <izz>200</izz>\n")
            f_new.write("          </inertia>\n")
            f_new.write("        </inertial>\n")
            f_new.write("        <collision name='collision'>\n")
            f_new.write("          <geometry>\n")
            f_new.write("            <box>\n")
            f_new.write("              <size>1 1 1</size>\n")
            f_new.write("            </box>\n")
            f_new.write("          </geometry>\n")
            f_new.write("          <max_contacts>10</max_contacts>\n")
            f_new.write("          <surface>\n")
            f_new.write("            <contact>\n")
            f_new.write("              <ode/>\n")
            f_new.write("            </contact>\n")
            f_new.write("            <bounce/>\n")
            f_new.write("            <friction>\n")
            f_new.write("              <torsional>\n")
            f_new.write("                <ode/>\n")
            f_new.write("              </torsional>\n")
            f_new.write("              <ode/>\n")
            f_new.write("            </friction>\n")
            f_new.write("          </surface>\n")
            f_new.write("        </collision>\n")
            f_new.write("        <visual name='visual'>\n")
            f_new.write("          <geometry>\n")
            f_new.write("            <box>\n")
            f_new.write("              <size>1 1 1</size>\n")
            f_new.write("            </box>\n")
            f_new.write("           </geometry>\n")
            f_new.write("          <material>\n")
            f_new.write("            <script>\n")
            f_new.write("              <name>Gazebo/Orange</name>\n")
            f_new.write("              <uri>file://media/materials/scripts/gazebo.material</uri>\n")
            f_new.write("            </script>\n")
            f_new.write("          </material>\n")
            f_new.write("        </visual>\n")
            f_new.write("        <self_collide>0</self_collide>\n")
            f_new.write("        <enable_wind>0</enable_wind>\n")
            f_new.write("        <kinematic>0</kinematic>\n")
            f_new.write("      </link>\n")
            f_new.write("    </model>\n")

        # Add spacing in between

        f_new.write("\n######################################################################## \n")
        f_new.write("####### Add within <state world_name='default'> ######################## \n")
        f_new.write("######################################################################## \n")

        # Add model def within main section of world
        for i in range(len(obs_list)):

            obs_cen_x = obs_list[i][0][0] + block_w/2
            obs_cen_y = obs_list[i][0][1] + block_d/2
            obs_cen_z = float(obs_list[i][3][2]/2)
            height = obs_list[i][3][2]

            f_new.write("      <model name='unit_box_%03i'>\n" % i)
            f_new.write("        <pose frame=''>%8.2f %8.2f %8.2f 0 0 0</pose>\n" % (obs_cen_x, obs_cen_y, obs_cen_z))
            f_new.write("        <scale>%i %i %i</scale>\n" % (block_w, block_d, height))
            f_new.write("        <link name='link'>\n")
            f_new.write("          <pose frame=''>%8.2f %8.2f %8.2f 0 0 0</pose>\n" % (obs_cen_x, obs_cen_y, obs_cen_z))
            f_new.write("          <velocity>0 0 0 0 -0 0</velocity>\n")
            f_new.write("          <acceleration>0 0 0 0 -0 0</acceleration>\n")
            f_new.write("          <wrench>0 0 0 0 -0 0</wrench>\n")
            f_new.write("        </link>\n")
            f_new.write("      </model>\n")

        f_new.close()

def plot_obstacles(obs_list, fig):
    """
    Add list of cube obstacles to figure
    :param obs_list: list of cube obstacles
    :param fig: figure to plot in
    :return:
    """

    for i in range(len(obs_list)):
        points = obs_list[i]

        # Add other 4 points
        p5 = [points[1][0],points[2][1],0]
        p6 = [points[1][0],points[1][1],points[3][2]]
        p7 = [points[2][0], points[2][1], points[3][2]]
        p8 = [points[1][0],points[2][1], points[3][2]]
        points.append(p5)
        points.append(p6)
        points.append(p7)
        points.append(p8)

        edges = [
            [points[0], points[1], points[4], points[2]],
            [points[0], points[1], points[5], points[3]],
            [points[3], points[5], points[7], points[6]],
            [points[6], points[7], points[4], points[2]],
            [points[0], points[2], points[6], points[3]],
            [points[1], points[4], points[7], points[5]]
        ]

        # Plot cube
        faces = Poly3DCollection(edges, linewidths=1, edgecolors='k')
        faces.set_facecolor((1, 0.5, 0, 0.3))

        fig.add_collection3d(faces)

        # Plot the points themselves to force the scaling of the axes
        # fig.scatter(points[:][0], points[:][1], points[:][2], s=0)

def check_collision(point, obs_list):
    """
    Check whether a point [x,y,z] is within any obstacles in obs_list
    :param point: point as list [x,y,z]
    :param obs_list: list of obstacles, each of which is list of vertices [p1, p2, p3, p4]
    :return: collision: boolean that is True if point collides with obstacles, or if out of bounds
    """
    # Init collision
    collision = False
    i = 0

    px = point[0]
    py = point[1]
    pz = point[2]

    # Check if point is out of bounds
    if px < 0 or px > x_max:
        collision = True
        return collision
    elif py < 0 or py > y_max:
        collision = True
        return collision
    elif pz < 0 or pz > z_max:
        collision = True
        return collision

    while i < len(obs_list):

        x_obs_min = obs_list[i][0][0]
        x_obs_max = x_obs_min + block_w

        y_obs_min = obs_list[i][0][1]
        y_obs_max = y_obs_min + block_d

        z_obs_min = 0
        z_obs_max = obs_list[i][3][2]

        if x_obs_min <= px <= x_obs_max:
            if y_obs_min <= py <= y_obs_max:
                if z_obs_min <= pz <= z_obs_max:
                    collision = True
                    return collision
        i += 1

    return collision

def find_nearest(goal_node, tree):
    """
    Return nearest node in tree to 'node'
    :param goal_node: node
    :param tree: tree (list) of nodes
    :return: ret_node: nearest node
    """

    ret_node = None                         # Init nearest node to be empty
    shortest_dist = x_max*y_max*z_max       # Init shortest_dist to be large number

    for i in range(len(tree)):
        node_i = tree[i]

        dist = math.sqrt((node_i.x - goal_node.x)**2 + (node_i.y - goal_node.y)**2 + (node_i.z - goal_node.z)**2)

        if dist < shortest_dist:
            shortest_dist = dist
            ret_node = node_i

    return ret_node


def local_plan(start_node, goal_node, obs_list, dist_step):
    """
    Return a new node (connected to start node) in direction of goal node
    :param start_node: start node
    :param goal_node: goal node to reach
    :return: new_node: node and motion from end node to start
    """

    # Calculate distance to goal
    dist = math.sqrt((goal_node.x - start_node.x)**2 + (goal_node.y - start_node.y)**2 + (goal_node.z - start_node.z)**2)

    # Calculate change in heading and direction
    far_phi = math.atan2(goal_node.y - start_node.y, goal_node.x - start_node.x)

    delta_x = goal_node.x - start_node.x
    delta_y = goal_node.y - start_node.y
    delta_z = goal_node.z - start_node.z

    # Check if goal is too far to be reached in this iteration
    if dist > dist_step:
        target_phi = far_phi

        # Scale commands so that total distance travelled is dist_step
        delta_x = dist_step/dist*delta_x
        delta_y = dist_step/dist*delta_y
        delta_z = dist_step/dist*delta_z

        x_cmd = start_node.x + delta_x
        y_cmd = start_node.y + delta_y
        z_cmd = start_node.z + delta_z

    else:
        target_phi = goal_node.phi
        x_cmd = goal_node.x
        y_cmd = goal_node.y
        z_cmd = goal_node.z

    # Check if target point is in free space
    if check_collision([x_cmd, y_cmd, z_cmd],obs_list):
        return None

    # Set phi command
    delta_phi = target_phi - start_node.phi

    # Limit phi within +/- limits
    if delta_phi > phi_lim:
        delta_phi = phi_lim
    elif delta_phi < -phi_lim:
        delta_phi = -phi_lim

    # Set phi command
    phi_cmd = start_node.phi + delta_phi

    # Limit phi between +/- pi
    if phi_cmd > math.pi:
        phi_cmd -= 2*math.pi
    elif phi_cmd < -math.pi:
        phi_cmd += 2*math.pi

    # Check that linear interpolation between start node and new node is collision free
    delta_dist = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

    x_last = start_node.x
    y_last = start_node.y
    z_last = start_node.z
    phi_last = start_node.phi
    motion_start = [x_last, y_last, z_last, math.degrees(phi_last)]
    motion = [motion_start]

    for i in range(int(math.floor(delta_dist))):
        x_last = start_node.x + i/delta_dist*delta_x
        y_last = start_node.y + i/delta_dist*delta_y
        z_last = start_node.z + i/delta_dist*delta_z
        phi_last = math.degrees(start_node.phi) + math.degrees(i/delta_dist*delta_phi)

        if check_collision([x_last, y_last, z_last],obs_list):
            return None
        else:
            motion.append([x_last,y_last,z_last,phi_last])

    new_cost = start_node.cost + delta_dist

    motion.reverse()
    new_node = Node(x_cmd, y_cmd, z_cmd, phi_cmd, start_node, motion, cost=new_cost)

    return new_node

def compute_inc_path(end_node):
    """
    Return incremental path from start node to end node (including lienar interpolation between nodes)
    :param end_node: end of path
    :return: path:
    """
    current_node = end_node
    path = []

    while current_node.parent != []:
        if current_node.p_edge != []:
            path = path + current_node.p_edge
        current_node = current_node.parent

    path.reverse()

    return path

def compute_node_path(end_node):
    """
    Return path from start node to end node consisting of nodes in between
    :param end_node: end of path
    :return: path:
    """
    current_node = end_node
    path = [[end_node.x, end_node.y, end_node.z, math.degrees(end_node.phi)]]

    while current_node.parent != []:
        path.append([current_node.parent.x, current_node.parent.y, current_node.parent.z, math.degrees(current_node.parent.phi)])
        current_node = current_node.parent

    path.reverse()

    return path

def update_costs(node):
    """
    Update all costs in path leading to node
    :param node:
    :return:
    """
    # Compute path cost to node1
    current_node = node
    cost_list = []

    while current_node.parent != []:
        cost_list.append(current_node.cost - current_node.parent.cost)
        current_node = current_node.parent

    current_node = node

    for i in range(len(cost_list)):
        current_node.cost = sum(cost_list[i:len(cost_list)])
        current_node = current_node.parent

class Node(object):
    def __init__(self, x, y, z, phi, parent, p_edge, cost=x_max*y_max*z_max):
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

def rrt_search(start,end,obs_list, fig):
    """
    Perform RRT search from start to end, navigating through list of obstacles
    :param start: start point [start_x, start_y, start_z, start_phi]
    :param end: end point [end_x, end_y, end_z, end_phi]
    :param obs_list: list of obstacles, each of which is list of cube vertices (4 out of 8)
    :param fig: figure to update
    :return: path: path from start to end. List of [x,y,z]
    """

    # Start timer
    ti = time.time()

    # Init nodes
    start_node = Node(start[0],start[1],start[2],start[3],[],[],cost=0)
    end_node = Node(end[0],end[1],end[2],end[3],[],[],cost=0)

    # Init tree
    tree = [start_node]
    all_edges = []

    while len(tree) < max_tree_size:

        # Randomly sample Cfree (with some probability, select end_node instead)
        if rand.random() < goal_weight:
            samp_node = end_node
        else:
            samp_free = False
            while not samp_free:
                samp_x = x_max*rand.random()
                samp_y = y_max*rand.random()
                samp_z = z_max*rand.random()

                if not check_collision([samp_x, samp_y, samp_z], obs_list):
                    samp_free = True

            samp_phi = math.pi*(2*rand.random()-1)
            samp_node = Node(samp_x,samp_y,samp_z,samp_phi,[],[])

        # Select nearest node in tree to sample from Cfree
        nearest_node = find_nearest(samp_node,tree)

        # Find motion from nearest node to new node in direction of sampled node
        dist_step = rand.random()*trans_lim
        new_node = local_plan(nearest_node, samp_node, obs_list,dist_step)

        # Check that path to new node is collision free
        if new_node != None:
            # Add new node to tree with nearest node as parent
            new_node.parent = nearest_node
            tree.append(new_node)

            all_edges.append(new_node.p_edge)

            # Dynamically plot edges

            #line = plt3d.art3d.Line3D([nearest_node.x, new_node.x], [nearest_node.y, new_node.y], [nearest_node.z, new_node.z], color='c', linewidth=1)
            #fig.add_line(line)
            #plt.pause(0.01)

            # Check if new_node is close enough to goal
            if (abs(new_node.x - end_node.x) < dx_tol) and (abs(new_node.y - end_node.y) < dy_tol) \
                    and (abs(new_node.z - end_node.z) < dz_tol) and (abs(new_node.phi - end_node.phi) < dphi_tol):
                print('SUCCESS!! RRT path found!')
                path = compute_node_path(new_node)

                # End timer, print execution time
                tf = time.time()
                print('RRT solution found in %s' % (tf - ti))

                # Compute solution distance
                update_costs(new_node)
                print('RRT solution cost: %s' % new_node.cost)

                return path, all_edges

        if len(tree) % 50 == 0:
            print(new_node)
            print(len(tree))

    print('FAILED :( no RRT path found')
    path = compute_node_path(new_node)
    return path, all_edges



def main():

    """
    Make new height_map
    """
    # m = make_height_map(x_blocks, y_blocks, h_map_file)

    """
    Make new 3D matrix
    """
    # mat = make_3d_mat(h_map_file)
    # print(mat[0][2][4])

    # cube1 = [(0, 0, 0), (0, 1, 0), (1, 0, 0), (0, 0, 3)]
    # plot_cube(cube1)

    """
    Make new obstacle list
    """
    obs_list = make_obs_list(h_map_file)
    # make_obs_models(obs_list,"obstacle_list_v5.txt")

    """
    Define start and end points
    """
    # Init start point to be 0,0,1,0
    start_point = [0, 0, 1, 0]

    # End in upper-right 20%
    end_free = False
    while not end_free:
        end_x = 0.75 * x_max + 0.2 * x_max * rand.random()
        end_y = 0.75 * y_max + 0.2 * y_max * rand.random()
        end_z = 1

        if not check_collision([end_x, end_y, end_z], obs_list):
            end_free = True

    end_phi = math.pi * (2 * rand.random() - 1)
    end_point = [end_x, end_y, end_z, end_phi]

    print(end_point)


    """
    Init figure plot
    """

    # Init figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Configure axes
    ax.set_xlim([0, x_max])
    ax.set_ylim([0, y_max])
    ax.set_zlim([0, max_block_height])

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Plot obstacles (city blocks)
    plot_obstacles(obs_list,ax)
    # plt.pause(0.01)

    # Plot start and end point
    ax.scatter(start_point[0], start_point[1], start_point[2], color='c', s=20)
    ax.scatter(end_point[0], end_point[1], end_point[2], color='m', s=20)


    """
    RRT search
    """
    raw_input('press enter to start RRT search')
    path, all_paths = rrt_search(start_point, end_point, obs_list, ax)


    # Plot results
    for i in range(len(all_paths)):
        line = plt3d.art3d.Line3D([item[0] for item in all_paths[i]], [item[1] for item in all_paths[i]],
                                  [item[2] for item in all_paths[i]], color='c', linewidth=1)
        ax.add_line(line)

    for i in range(len(path)-1):

        line = plt3d.art3d.Line3D([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]],
                                  [path[i][2], path[i+1][2]], color='lime', linewidth=1)
        ax.add_line(line)

    # Plot final plot
    plt.pause(0.01)
    raw_input('press enter to finish')
    plt.show()




if __name__ == "__main__":
    main()

