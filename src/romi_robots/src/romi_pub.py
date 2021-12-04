################################################################################
#
# Node to wrap the romi class.
#
################################################################################

from re import X
import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# YDLIDAR SPECIFICATIONS
ANGLE_MIN = 0.0
ANGLE_MAX = 360.0
ANGLE_INCREMENT = 0.01
TIME_INCREMENT = 0.1
SCAN_TIME = 0.8
RANGE_MIN = 0.12
RANGE_MAX = 10.0
INTENSITIES = []

# ROMI ROBOT PHYSICAL SPECIFICATIONS
WHEEL_TRACK = 11.2
WHEEL_RADIUS = 0.36
TICKS_PER_ROTATION = 351

class Romi():
    def __init__(self, name):
        # romi's name
        self.name = name
        # romi's position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # romi's velcity
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        # romi's old wheel encoder data
        self.old_left_ticks = 0.0
        self.old_right_ticks = 0.0
        # romi's old time
        self.old_time = rospy.Time.now()
        # broadcasting and publishing objects
        self.laser_pub = rospy.Publisher("scan", LaserScan, queue_size=50)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

    def publish_laser(self, range_data):
        # Publish to laser topic
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = ANGLE_MIN
        scan.angle_max - ANGLE_MAX
        scan.angle_increment = ANGLE_INCREMENT
        scan.time_increment = TIME_INCREMENT
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        scan.ranges = []
        scan.intensities = []
        for r in range_data:
            scan.ranges.append(r)
        self.laser_pub.publish(scan)
            

    def broadcast_and_publish_odom(self, left_tick_data, right_tick_data):
        current_time = rospy.Time.now()
        delta_left = left_tick_data - self.old_left_ticks
        delta_right = right_tick_data - self.old_right_ticks
        dl = 2 * np.pi * delta_left / TPR
        dr = 2 * np.pi * delta_right / TPR
        dc = (dl + dr) / 2
        dt = (current_time - self.old_time)
        dth = (dr - dl) / WHEEL_TRACK

        if dl == dr:
            dx = dr * np.cos(self.theta)
            dy = dr * np.sin(self.theta)
        else:
            radius = dc / dth
            iccX = self.x - radius*np.sin(self.theta)
            iccY = self.y + radius*np.cos(self.theta)
            dx = np.cos(dth) * (self.x - iccX) - np.sin(dth) * (self.y - iccY) + iccX - self.x
            dy = np.sin(dth) * (self.x - iccX) - np.cos(dth) * (self.y - iccY) + iccY - self.y

        self.x += dx
        self.y += dy
        self.theta = (self.theta + dth) % (2 * np.pi)

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        self.odom_broadcaster.sendTransform((self.x, self.y, 0.), odom_quat, current_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        if dt.to_sec() > 0.0:
            self.vx = dx/(dt.to_sec())
            self.vy = dy/(dt.to_sec())
            self.omega = dth/(dt.to_sec())
        
        odom.child_frame_id = 'base_link'
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.omega))
        print(odom)
        self.odom_pub.publish(odom)

        # Record current left and right ticks
        # and current time for next iteration
        self.old_left_ticks = left_tick_data
        self.old_right_ticks = right_tick_data
        self.old_time = current_time