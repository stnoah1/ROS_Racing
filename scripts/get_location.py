#!/usr/bin/env python

## Course: ECET581 Fall2018 
## Authors: Hyung-gun Chi and Min Huang 
## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped

# Define a wrapper for callback
def mapCallback(data):
    global map_data
    global map_flag
    map_data = data
    map_flag = True


def amclCallback(data):
    global amcl_data
    global amcl_flag
    amcl_data = data
    amcl_flag = True


def mapMetaCallback(data):
    global map_meta_data
    global map_meta_flag
    map_meta_data = data
    map_meta_flag = True
 
def main():
    global scan_data
    global map_meta_data
    global map_meta_flag
    global map_data
    global map_flag
    global amcl_data
    global amcl_flag
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    # Initialization
    rospy.init_node('amcl_test', anonymous=True)
    
    # Subscribe to topic 'scan'
    sub_map_meta = rospy.Subscriber('map_metadata', MapMetaData, mapMetaCallback)
    sub_map = rospy.Subscriber('map', OccupancyGrid, mapCallback)
    sub_amcl_pose = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amclCallback)

    map_meta_flag = False
    map_flag = False
    amcl_flag = True    
      
    while not rospy.is_shutdown():
        if map_meta_flag and map_flag and amcl_flag:
            map_height = map_meta_data.height
            map_width = map_meta_data.width
            map_resolution = map_meta_data.resolution
            org_x = map_meta_data.origin.position.x
            org_y = map_meta_data.origin.position.y
            amcl_pose = amcl_data.pose.pose.position
            r = int((amcl_pose.x - org_x)/map_resolution)
            c = int((amcl_pose.y - org_y)/map_resolution)
            one_d_map = np.array(map_data.data)
            print(one_d_map[r*map_width + c])
            print(map_data)
            break
    rospy.spin()


if __name__ == '__main__':
    main()
