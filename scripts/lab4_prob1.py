#!/usr/bin/env python

## Course: ECET581 Fall2018 
## Authors: Hyung-gun Chi and Min Huang 
## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int8MultiArray

# Define a wrapper for callback
def scanCallback(data):
    print('test')
    print(data.data)

def scanListener():
    global scan_data
    global flag
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    # Initialization
    rospy.init_node('amcl', anonymous=True)
    
    # Subscribe to topic 'scan'
    sub = rospy.Subscriber('map', OccupancyGrid, scanCallback)
    # Initialize the flag
    print("Let's start ... ")
    rospy.spin()


if __name__ == '__main__':
    scanListener()
