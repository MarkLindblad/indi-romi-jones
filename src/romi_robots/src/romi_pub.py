#!/usr/bin/python
################################################################################
#
# Node to wrap the OccupancyGrid2d class.
#
################################################################################

from _typeshed import Self
from numpy.lib.function_base import angle
import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs import Odometry
import geometry_msgs
from romi import Romi

# YDLIDAR SPECIFICATIONS
ANGLE_MIN = 0.0
ANGLE_MAX = 360.0
RANGE_MIN = 0.0
RANGE_MAX = 0.0

# ROMI ROBOT PHYSICAL SPECIFICATIONS
WHEEL_TRACK = 0.0
WHEEL_RADIUS = 0.0
TPR = 351

class Romi():
    def __init__(self, name):
        # romi's name
        self.name = name
        # romi's old wheel encoder data
        self.old_left_ticks = 0.0
        self.old_right_ticks = 0.0
        # broadcasting and publishing objects
        self.laser_pub = rospy.Publisher("laser", LaserScan, queue_size=10)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

    def publish_laser(self, range, angle, time_stamp):
        # Publish to laser topic
        scan = LaserScan()
        scan.header.stamp = time_stamp
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = ANGLE_MIN
        scan.angle_max - ANGLE_MAX
        scan.angle_increment = angle
        scan.time_increment = 0.0
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
            

    def broadcast_and_publish_odom(self, left_tick_data, right_tick_data):
        change_in_left_ticks = left_tick_data - self.old_left_ticks
        cahnge_in_right_ticks = right_tick_data - self.old_right_ticks

        

            
