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
    #connect to the server on local computer
    sock.connect((_IP_ADDRESS, _PORT))
    #initialize ROS node 
    rospy.init_node(romi_name, anonymous=True)
    # instantiate romi object
    romi = Romi(romi_name)
    count = 0
    ranges = []
    #time_stamps = []
    while not rospy.is_shutdown():
        #Receive sensor data
        time_data = struct.unpack('f', sock.recv(4))[0]
        range_data = struct.unpack('f', sock.recv(4))[0] 
        angle_data = struct.unpack('f', sock.recv(4))[0]
        left_tick_data = struct.unpack('i', sock.recv(4))[0]
        right_tick_data = struct.unpack('i', sock.recv(4))[0]
        ranges.append(range_data)
        #time_stamps.append(time_data)
        #Print for debugging
        print("###############################\n")
        print("Laser Range: \n", range_data)
        print("Laser Angle \n", angle_data)
        print("Laser Time Stamp: \n", time_data)
        print("Left Ticks : \n", left_tick_data)
        print("Right Ticks: ", right_tick_data)
        print("###############################\n")
        if count == 360:
            # publish laser data
            romi.publish_laser(ranges)
            count = 0
            ranges = []
            #time_stamps = []
        # publish odom data
        romi.broadcast_and_publish_odom(0, 0)
        count += 1
        

if __name__ == '__main__':
    try:
        # sys.argv contains romi info
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass


       
 
            
 

 
