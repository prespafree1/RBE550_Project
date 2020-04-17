# Derek Larson
# Final Project
# 4/15/2020


import numpy
import os


class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

# The function creates a shell script of waypoints given an array of points (x, y, z).
def create_waypoint_file(waypoints):
    # delete waypoint_script if exists, since going to append to it
    if os.path.exists("waypoint_script.sh"):
        os.remove("waypoint_script.sh")
    # create waypoint_script.sh, and append waypoints.
    f = open('waypoint_script.sh', 'a')
    for i in waypoints:
        numpy.savetxt(f, ["rosrun rotors_gazebo waypoint_publisher " + str(i.x) +
                          " " + str(i.y) + " " + str(i.z) + " 0 0 __ns:=firefly"], fmt='%s')
    f.close()
    os.chmod("waypoint_script.sh", 0o777)

    return


if __name__ == "__main__":

    p1 = Point(1, 0, 1)
    p2 = Point(2, 3, 1)
    p3 = Point(5, -8, 1)
    p4 = Point(1, 4, 3)
    p5 = Point(-2, -4, 2)
    p6 = Point(0, 0, 0)
    waypoints = [p1, p2, p3, p4, p5, p6]
    create_waypoint_file(waypoints)