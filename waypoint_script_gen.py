# Derek Larson
# Final Project
# 4/15/2020


import numpy
import os


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
        numpy.savetxt(f, ["rosrun rotors_gazebo waypoint_publisher " + str(i.x) +
                          " " + str(i.y) + " " + str(i.z) + " " + str(i.phi) + " 0 __ns:=firefly"], fmt='%s')
    f.close()
    os.chmod("waypoint_script.sh", 0o777)

    return


if __name__ == "__main__":

    p1 = Node(1, 0, 1,0,[],[],cost=0)
    p2 = Node(2, 3, 1,0,[],[],cost=0)
    p3 = Node(5, -8, 1,5,[],[],cost=0)
    p4 = Node(1, 4, 3,0,[],[],cost=0)
    p5 = Node(-2, -4, 2,1,[],[],cost=0)
    p6 = Node(0, 0, 0,0,[],[],cost=0)
    waypoints = [p1, p2, p3, p4, p5, p6]
    create_waypoint_file(waypoints)