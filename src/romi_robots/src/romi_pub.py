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
ANGLE_MIN = -3.141593
ANGLE_MAX = 3.141593
ANGLE_INCREMENT = 0.012849
TIME_INCREMENT = 0.000250
SCAN_TIME = 0.122000
RANGE_MIN = 0.100000
RANGE_MAX = 10.000000
INTENSITIES = []

# ROMI ROBOT PHYSICAL SPECIFICATIONS
WHEEL_TRACK = 0.149
WHEEL_RADIUS = 0.036
# 72 * pi/1000*(1/0.0006108)
TICKS_PER_ROTATION = 370

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
        self.old_left_ticks = 0
        self.old_right_ticks = 0
        # romi's old time
        self.old_time = rospy.Time.now()
        # lidar ranges
        self.ranges = []
        # broadcasting and publishing objects
        self.laser_pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # self.ranges = []
        # self.left_tick_data = 0
        # self.right_tick_data = 0

    # def set_data(self, range, left, right):
    #     self.ranges.append(range)
    #     self.left_tick_data = left
    #     self.right_tick_data = right
    

    def publish_sensor_data(self, ranges, left_tick_data, right_tick_data):
        current_time = rospy.Time.now()

        # Publish to laser topic
        scan = LaserScan()
        scan.header.stamp = current_time
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = ANGLE_MIN
        scan.angle_max = ANGLE_MAX
        scan.angle_increment = ANGLE_INCREMENT
        scan.time_increment = TIME_INCREMENT
        scan.scan_time = SCAN_TIME
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        scan.ranges = []
        scan.intensities = []
        for r in ranges:
            scan.ranges.append(r)
        self.laser_pub.publish(scan)

        ##########################ODOMETRY##################################333
        delta_left = self.get_forward_tick_delta(left_tick_data, self.old_left_ticks)
        delta_right = self.get_forward_tick_delta(right_tick_data, self.old_right_ticks)
        # print("Left Encoder: \n", left_tick_data, self.old_left_ticks)
        # print("Right Encoder: \n", right_tick_data, self.old_right_ticks)
        dl = 2 * np.pi * delta_left / TICKS_PER_ROTATION
        dr = 2 * np.pi * delta_right / TICKS_PER_ROTATION
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
        self.odom_pub.publish(odom)

        # Record current left and right ticks
        # and current time for next iteration
        self.old_left_ticks = left_tick_data
        self.old_right_ticks = right_tick_data
        self.old_time = current_time

    
            
    def get_forward_tick_delta(self, new_tick, old_tick):
        # CONVERSION = 0.0006108  for getting distance, we only need the delta_tick !!!!!!
        """
        if abs(new_tick - old_tick) > (1<<15): 
            return CONVERSION*((1<<16) - abs(new_tick - old_tick))
        return abs(CONVERSION*(new_tick - old_tick))
        """
        if new_tick < old_tick:
            #print("============== diff:", (1 << 16) - old_tick + new_tick )
            return (1 << 16) - old_tick + new_tick 
        else:
            #print("============== reg:", new_tick - old_tick, new_tick)
            return (new_tick - old_tick)
