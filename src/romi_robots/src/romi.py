#!/usr/bin/env python3
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import sys
from indi_romi import Romi
from romi_state import State
from std_msgs.msg import String

###############################SOCKET###################################
# Import socket module to read sensor data
# from romi robots via WIFI
import socket            
import struct

# Port on which you want to connect
_PORT = 8080
# IP Address
_IP_ADDRESS = '172.20.10.6'
# socket object
_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#########################################################################

  

def main(args):
    # get romi name
    romi_name = args[1]
    #initialize ROS node 
    rospy.init_node(romi_name, anonymous=True)
    # instantiate romi object
    romi = Romi(romi_name)
    # subscribe to direction topic from path_planner
    # rospy.Subscriber('direction', String, romi.callback)
    #connect to the server on local computer
    _SOCKET.connect((_IP_ADDRESS, _PORT))
   
   
    count = 0
    ranges = []
    avg_ranges = []
    # Start State Machine
    while not rospy.is_shutdown():
        if romi.state == State.Receive:
            # Receive Sensor Data from Romi
            new_time_stamp = struct.unpack('i', _SOCKET.recv(4))[0]
            range_data = struct.unpack('f', _SOCKET.recv(4))[0] 
            angle_data = struct.unpack('f', _SOCKET.recv(4))[0]
            left_tick_data = struct.unpack('i', _SOCKET.recv(4))[0]
            right_tick_data = struct.unpack('i', _SOCKET.recv(4))[0]
            avg0 = struct.unpack('f', _SOCKET.recv(4))[0]
            avg1 = struct.unpack('f', _SOCKET.recv(4))[0]
            avg2 = struct.unpack('f', _SOCKET.recv(4))[0]
            avg3 = struct.unpack('f', _SOCKET.recv(4))[0]
            avg4 = struct.unpack('f', _SOCKET.recv(4))[0]
            print("Count: ", count)
            print("Avg Range0: \n", avg0)
            print("Avg Range1: \n", avg1)
            print("Avg Range2: \n", avg2)
            print("Avg Range3: \n", avg3)
            print("Avg Range4: \n", avg4)
            ranges.append(range_data)
            # Publish and broadcast when time stamp changes
            if count == 430:
                avg_ranges.extend([avg0, avg1, avg2, avg3, avg4])
                # publish laser and wheel encoder data
                romi.publish_sensor_data(ranges, left_tick_data, right_tick_data, avg_ranges)
                avg_ranges = []
                ranges = []
                count = 0
            count += 1
        
        elif romi.state == State.Send:
            # This state uses data coming from a node in the exploration package
            # Drive state
            if romi.direction == "Forward":
                direction = 'F'
                _SOCKET.send(direction.encode())
            elif romi.direction == "Right":
                direction = 'R'
                _SOCKET.send(direction.encode())
            romi.state = State.Receive



if __name__ == '__main__':
    try:
        # sys.argv contains romi info
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass


       
 
            
 

 
