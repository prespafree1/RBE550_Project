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
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

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

h_map_file = "height_map2.txt"   # City block height map .txt file to use


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
        faces.set_facecolor((1, 0.5, 0, 1))

        fig.add_collection3d(faces)

        # Plot the points themselves to force the scaling of the axes
        # fig.scatter(points[:][0], points[:][1], points[:][2], s=0)

def main():

    """
    Make new height_map
    """
    # m = make_height_map(x_blocks, y_blocks, h_map_file)

    """
    Make new 3D matrix
    """
    # m = make_3d_mat(h_map_file)
    # print(m)

    # cube1 = [(0, 0, 0), (0, 1, 0), (1, 0, 0), (0, 0, 3)]
    # plot_cube(cube1)

    """
    Make new obstacle list
    """
    obs_list = make_obs_list(h_map_file)
    # make_obs_models(obs_list,"obstacle_list_v5.txt")

    """
    Init figure plot
    """

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([0, x_blocks*block_w + (x_blocks + 1)*street_w])
    ax.set_ylim([0, y_blocks*block_d + (y_blocks + 1)*street_w])
    ax.set_zlim([0, 1.5*max_block_height])

    plot_obstacles(obs_list,ax)

    plt.show()













if __name__ == "__main__":
    main()

