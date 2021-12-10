#!/usr/bin/env python3
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import sys
from romi_pub import Romi
import numpy as np
# Import socket module to read sensor data
# from romi robots via WIFI
import socket            
import struct

# Port on which you want to connect
_PORT = 8080
# IP Address
_IP_ADDRESS = '172.20.10.6'  

# TODO Romi() requires sensor data
def main(args):
    # get romi name
    romi_name = args[1]
    # Create a socket object
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #Subrcribe to command topic
    #connect to the server on local computer
    sock.connect((_IP_ADDRESS, _PORT))
    #initialize ROS node 
    rospy.init_node(romi_name, anonymous=True)
    # instantiate romi object
    romi = Romi(romi_name)
    count = 0
    # flag
    ranges = []
    #rospy.Timer(rospy.Duration(3), romi.publish_sensor_data)
    
    while not rospy.is_shutdown():
        # Receive Sensor Data from Romi
        new_time_stamp = struct.unpack('i', sock.recv(4))[0]
        range_data = struct.unpack('f', sock.recv(4))[0] 
        angle_data = struct.unpack('f', sock.recv(4))[0]
        left_tick_data = struct.unpack('i', sock.recv(4))[0]
        right_tick_data = struct.unpack('i', sock.recv(4))[0]
        ranges.append(range_data)
        #Publish and broadcast when time stamp changes
        if count == 430:
            # publish laser and wheel encoder data
            romi.publish_sensor_data(ranges, left_tick_data, right_tick_data)
            ranges = []
            count = 0
        count += 1

        #romi.set_data(range_data, left_tick_data, right_tick_data)


if __name__ == '__main__':
    try:
        # sys.argv contains romi info
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass


       
 
            
 

 
