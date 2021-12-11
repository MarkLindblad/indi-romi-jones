#!/usr/bin/python3

import sys
import rospy
import numpy as np
import traceback

from path_planner import PathPlanner
    
def main():
    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("romi1")


    #TODO Add range contraints for collision avoidance

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    rospy.init_node('path')
    main()