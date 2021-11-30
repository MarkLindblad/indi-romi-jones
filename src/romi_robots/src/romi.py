#!/usr/bin/env python3
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
from re import S
import rospy
import tf2_py
import tf2_ros
import tf2_msgs.msg
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
import geometry_msgs
import asyncio
from bleak import BleakClient
from romi_pub import Romi

def main(args):
    romi_name = args[1]
    rospy.init_node(romi_name, anonymous=True)
    romi = Romi()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    # sys.argv contains romi info
    main(sys.argv)


# THIS CODE IS FOR BLE COMMUNICATION WITH THE ROMI ROBOTS
#####################################################################
# from ble_utils import parse_ble_args, handle_sigint, LAB11
# args = parse_ble_args('Print advertisement data from a BLE device')
# addr = args.addr.lower()
# timeout = args.timeout
# handle_sigint()


# ROMI1_ADDRESS = hex(0x1234)

# async def main(address):
#     print(f"searching for device {address} ({timeout}s timeout)")
#     async with BleakClient(address,timeout=timeout) as client:
#         print(f"Connected: {client.is_connected}")        
#         for service in client.services:
#             print(f"[Service] {service}")
#             for char in service.characteristics:
#                 if "read" in char.properties:
#                     try:
#                         value = bytes(await client.read_gatt_char(char.uuid))
#                         print(f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {value}")
#                     except Exception as e:
#                         print(f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {e}")
#                 else:
#                     value = None
#                     print(f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {value}")

#                 for descriptor in char.descriptors:
#                     try:
#                         value = bytes(await client.read_gatt_descriptor(descriptor.handle))
#                         print(f"\t\t[Descriptor] {descriptor}) | Value: {value}")
#                     except Exception as e:
#                         print(f"\t\t[Descriptor] {descriptor}) | Value: {e}")


# if __name__ == "__main__":
#     while True:
#         asyncio.run(main(ROMI1_ADDRESS))
