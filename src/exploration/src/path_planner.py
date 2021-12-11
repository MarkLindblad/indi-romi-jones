# TODO determine a direction for indi romi to follow. To accomplish this, we'll
# need to use a variant/combination of A*, BFS/DFS, random walk algorithm and 
# forward odometry kinematics

# Naive Maze Algorithm

"""
Path Planner Class
"""

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class PathPlanner(object):
    """
    Path Planning Functionality for Baxter/Sawyer

    We make this a class rather than a script because it bundles up 
    all the code relating to planning in a nice way thus, we can
    easily use the code in different places. This is a staple of
    good object-oriented programming

    Fields:
    _robot: moveit_commander.RobotCommander; for interfacing with the robot
    _scene: moveit_commander.PlanningSceneInterface; the planning scene stores a representation of the environment
    _group: moveit_commander.MoveGroupCommander; the move group is moveit's primary planning class
    _planning_scene_publisher: ros publisher; publishes to the planning scene


    """
    def __init__(self, romi_name):
        self.name = romi_name
        rospy.Subscriber('pose', Pose, self.pose_callback)
        rospy.Subscriber('avg_scan', LaserScan, self.laser_callback)
        self.direction_pub = rospy.Publisher('direction', String, queue_size=10)

        self.x = 0
        self.y = 0
        self.avg_ranges = []

    def pose_callback(self, msg):
        pose = msg.data
        self.x = pose.position.x
        self.y = pose.position.y
    
    def laser_callback(self, msg):
        self.avg_ranges = msg.data.ranges

    def direction_to_pose(self):
        """
        Generates a direction given a pose with respect to a fixed frame subject to lidar range contraints
        """
        if 0.5 in self.avg_ranges:
            self.direction_pub.publish("Right")
        else:
            self.direction_pub.publish("Forward")

