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
# Import socket module to read sensor data
# from romi robots via WIFI
import socket            
import struct 


# Port on which you want to connect
_PORT = 8080
# IP Address
_IP_ADDRESS = '192.168.137.22' 

# TODO Romi() requires sensor data
def main(args):
    # get romi name
    romi_name = args[1]
    # Create a socket object
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # connect to the server on local computer
    sock.connect((_IP_ADDRESS, _PORT))
    # initialize ROS node 
    rospy.init_node(romi_name, anonymous=True)
    # instantiate romi object
    romi = Romi(romi_name)
    while True:
        # Receive sensor data
        range_data = struct.unpack('f', sock.recv(4))[0] 
        angle_data = struct.unpack('f', sock.recv(4))[0]
        time_data = struct.unpack('f', sock.recv(4))[0]
        left_tick_data = struct.unpack('f', sock.recv(4))[0]
        right_tick_data = struct.unpack('f', sock.recv(4))[0]
        gyro_data = struct.unpack('f', sock.recv(4))[0]
        # Print for debugging
        print("Range: \n", range_data)
        print("Angle: \n", angle_data)
        print("Left Ticks : \n", left_tick_data)
        print("Right Ticks: ", right_tick_data)
        print("Gyro Data: \n", gyro_data)
        # publish laser data
        romi.publish_laser(range_data, angle_data, time_data)
        # publish odom data
        romi.broadcast_and_publish_odom(left_tick_data, right_tick_data, gyro_data)
        

if __name__ == '__main__':
    # sys.argv contains romi info
    main(sys.argv)


       
 
            
 

 
