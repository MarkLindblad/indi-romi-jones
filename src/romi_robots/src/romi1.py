#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_py
import tf2_ros
import tf2_msgs.msg
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
import geometry_msgs

# This is for publishing tf frames.
# t = geometry_msgs.msg.TransformStamped()
# t.header.frame_id = CAMERA_FRAME
# t.header.stamp = rospy.Time.now()
# t.child_frame_id = TAG_FRAME.format(id)
# t.transform.translation.x = tvec[0]
# t.transform.translation.y = tvec[1]
# t.transform.translation.z = tvec[2]

# t.transform.rotation.x = q[0]
# t.transform.rotation.y = q[1]
# t.transform.rotation.z = q[2]
# t.transform.rotation.w = q[3]


# tfm = tf2_msgs.msg.TFMessage([t])
# self.frame_pub.publish(tfm)

# topic_image = self.bridge.cv2_to_imgmsg(cv_grayImage)
# self.image_pub.publish(topic_image)


def main():
    rospy.init_node("romi1")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()