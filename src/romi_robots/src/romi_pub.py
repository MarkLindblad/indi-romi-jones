#!/usr/bin/python
################################################################################
#
# Node to wrap the OccupancyGrid2d class.
#
################################################################################

import rospy
import tf2_py
import tf2_ros
import tf2_msgs.msg
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
import geometry_msgs
from romi import Romi

class Romi():
    def __init__(self, romi_name, laser_topic, odom_topic):
        self.name = romi_name

    
    def publish(range_data, angle_data, wheel_encoder, gyro):
        laserScan_pub = rospy.Publisher('laser_point', LaserScan, queue_size=10)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            scan = LaserScan()
            scan.header.stamp = current_time
            scan.header.frame_id = 'laser_frame'
            scan.angle_min = 0.0
            scan.angle_max - 0.0
            scan.angle_increment = 0.0
            scan.time_increment = 0.0
            scan.range_min = 0.0
            scan.range_max = 0.0
