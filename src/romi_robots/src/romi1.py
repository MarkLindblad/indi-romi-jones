#!/usr/bin/env python3
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
import asyncio
from bleak import BleakClient

from ble_utils import parse_ble_args, handle_sigint, LAB11
args = parse_ble_args('Print advertisement data from a BLE device')
addr = args.addr.lower()
timeout = args.timeout
handle_sigint()


ROMI1_ADDRESS = hex(0x1234)

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

async def main(address):
    print(f"searching for device {address} ({timeout}s timeout)")
    async with BleakClient(address,timeout=timeout) as client:
        print(f"Connected: {client.is_connected}")        
        for service in client.services:
            print(f"[Service] {service}")
            for char in service.characteristics:
                if "read" in char.properties:
                    try:
                        value = bytes(await client.read_gatt_char(char.uuid))
                        print(f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {value}")
                    except Exception as e:
                        print(f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {e}")
                else:
                    value = None
                    print(f"\t[Characteristic] {char} ({','.join(char.properties)}), Value: {value}")

                for descriptor in char.descriptors:
                    try:
                        value = bytes(await client.read_gatt_descriptor(descriptor.handle))
                        print(f"\t\t[Descriptor] {descriptor}) | Value: {value}")
                    except Exception as e:
                        print(f"\t\t[Descriptor] {descriptor}) | Value: {e}")


if __name__ == "__main__":
    while True:
        asyncio.run(main(ROMI1_ADDRESS))