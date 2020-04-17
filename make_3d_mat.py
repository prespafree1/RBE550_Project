"""
RBE550, Project Group 1

@author: Jesse Morzel
"""

"""
Define imports
"""
import random as rand
import math

'''
Define global_variables
'''
x_blocks = 10           # Number of city blocks in x-dimension
y_blocks = 10           # Number of city blocks in y-dimension
min_block_height = 5    # Min height of a block (m)
max_block_height = 100  # Max height of a block (m)

block_w = 50            # Depth of each block (m)
block_d = 50            # Width of each block (m)
street_w = 20           # Width of streets (m)

cell_res = 10           # Cell resolution for 3D-matrix (m)

h_map_file = "height_map.txt"   # City block height map .txt file to use


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



def main():

    # Make new height_mat
    # m = make_height_map(x_blocks, y_blocks, h_map_file)

    m = make_3d_mat(h_map_file)
    print(m)



if __name__ == "__main__":
    main()

