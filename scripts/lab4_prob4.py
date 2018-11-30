#!/usr/bin/env python

## Course: ECET581 Fall2018 
## Authors: Hyung-gun Chi and Min Huang and Donghun Lee 
## This is program for lab4 problem 4.


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
def curLocCallback(data):
    global curLoc_data
    global curLoc_flag
    curLoc_data = data
    curLoc_flag = True


def wayPntCallback(data):
    global wayPnt_data
    global wayPnt_flag
    wayPnt_data = data
    wayPnt_flag = True

def PD_control_speed(speed, prev_error):
    k_p = 10
    k_d = 80
    v_target = 200
    error = v_target - speed
    control_error = k_p*error + k_d*(error-prev_error)
    speed = speed + control_error
    # TODO: make some upper and lower bound for speed 
    return speed, error

def PD_control_angle(angle, prev_angle, prev_error): 
    k_p = 10
    k_d = 80
    error = angle - prev_angle
    control_angle = k_p*error + k_d*(error-prev_error)
    # TODO: make some upper and lower bound for angle
    angle = angle + control_angle 

    return angle, error
 
def send_serial(signal="A-0500+0319"):
    #initialize serial port to communicate with Tri-Car 
    #We are using a USB to RS232 converter on ttyUSB0 for Tri-Car
    #the port can be changed if there are multiple USB devices
    #initialize serial port to communicate with Rally Car 
    #We are using direct USB on ttyACM0 for Rally Car
    console_ser = serial.Serial("/dev/ttyUSB0",baudrate=115200)
    # construct and send the Ackermann steering commands to Car
    console_ser.write(signal)

def main():
    global curLoc_data
    global curLoc_flag
    global wayPnt_data
    global wayPnt_flag
    wayPnt_flag = False
    curLoc_flag = False

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    # Initialization
    rospy.init_node('amcl_test', anonymous=True)
    
    # Subscribe to topic 'scan'
    sub_current_location = rospy.Subscriber('current_location', String, curLocCallback)
    sub_way_point = rospy.Subscriber('way_point', String, wayPntCallback)
      
    speed_error = 0
    angle_error = 0
    prev_angle = 0
    while not rospy.is_shutdown():
        if wayPnt_flag and curLoc_flag:
           way_point_x, way_point_y = wayPnt_data
           current_location_x, current_location_y = curLoc_data
           current_angle = np.arctan2(way_point_y-current_location_y, way_point_x - current_location_x ) 
           PD_control_speed(current_speed, speed_error)
           PD_control_angle(current_angle, prev_angle, angle_error)
           current_location_x, current_location_y = curLoc_data
           send_serial()
    rospy.spin()


if __name__ == '__main__':
    main()
