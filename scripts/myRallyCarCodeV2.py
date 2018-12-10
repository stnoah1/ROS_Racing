#!/usr/bin/env python
#
## Course: ECET581 Fall2018
## Authors: Hyung-gun Chi and Donghun Lee
## This file subscribe the topic 'imu_info' from imu seirial, 'cur_pos' from amcl
## and PD control the rallycar based on these informations.

import csv
import time
from math import pow, atan2, sqrt, cos, sin, pi

import rospy
import serial
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float32


class RallyCar:
  def __init__(self, way_points):
    # Creates a node with name 'rallycar_controller'.
    print('initialize params')
    rospy.init_node('rallycar_controller', anonymous=True)

    # Set up constants
    self.distance_tolerance = 1.8
    self.angular_vel_Kp = 1000
    self.angular_vel_Kd = 400  # 40
    self.linear_vel_Kp = 100
    self.linear_vel_Kd = 80  # 100

    self.bias = 430
    self.freq = 30

    # initialize parameters
    self.rate = rospy.Rate(self.freq)
    self.way_points = way_points

    self.prev_amcl_pose_x = 0
    self.prev_amcl_pose_y = 0
    self.prev_amcl_theta = 0
    self.prev_amcl_stamp = time.time()

    self.amcl_pose_x = 0
    self.amcl_pose_y = 0
    self.amcl_theta = 0
    self.amcl_stamp = time.time()

    self.imu_stamp = time.time()

    self.pose_x = 0
    self.pose_y = 0
    self.theta = 0

    self.prev_angular_vel_error = 0
    self.prev_linear_vel_error = 0
    self.angular_vel_error = 0
    self.linear_vel_error = 0

    self.estimate_linear_vel = 0
    self.estimate_angular_vel = 0

    self.linear_vel = 0
    self.angular_vel = 0

    # A subscriber to the topic 'cur_pos'. self.update_pose is called when a message of type String is received.
    self.pose_subscriber = rospy.Subscriber('/cur_pos', String, self.update_pose)

    self.console_ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
    self.console_ser.close()
    self.console_ser.open()

    # A subscriber to the topic 'imu_info'. self.update_angular_vel is called when a message of type Float is received.
    self.imu_info_subscriber = rospy.Subscriber('/imu_info', Float32, self.update_angular_vel)

    # Initialize steering
    self.send_serial(self.bias, 0)
    time.sleep(1)

  def update_pose(self, data):
    """Callback function which is called when a new message of type current position is received by the subscriber."""
    self.prev_amcl_pose_x = self.amcl_pose_x
    self.prev_amcl_pose_y = self.amcl_pose_y
    self.prev_amcl_theta = self.amcl_theta
    self.prev_amcl_stamp = self.amcl_stamp

    pose = data.data.split(";")
    self.amcl_pose_x = float(pose[0])
    self.amcl_pose_y = float(pose[1])
    self.amcl_theta = float(pose[2])

    self.pose_x = self.amcl_pose_x
    self.pose_y = self.amcl_pose_y
    self.theta = self.amcl_theta
    self.amcl_stamp = time.time()

    time_delta = self.amcl_stamp - self.prev_amcl_stamp
    # print(self.amcl_pose_x, self.prev_amcl_pose_x)
    distant_delta = sqrt(pow((self.pose_x - self.prev_amcl_pose_x), 2) + pow((self.pose_y - self.prev_amcl_pose_y), 2))

    self.estimate_linear_vel = distant_delta / time_delta

    self.linear_vel = self.estimate_linear_vel
    print("amcl update!!!", self.pose_x, self.pose_y, self.theta)

    # rospy.loginfo("Current AMCL pose: x=" + str(self.pos_x) + "/ y=" + str(self.pos_y) + "/ theta=" + str(self.theta))

  def update_angular_vel(self, data):
    """update angular velocity with IMU data"""
    self.angular_vel = self.angular_vel + float(data.data) * (time.time() - self.imu_stamp)
    self.imu_stamp = time.time()

  def interpolate_linear_vel(self):
    """Interpolate the linear velocity depend on last two AMCL points and angular velocity"""
    time_delta = time.time() - self.amcl_stamp
    # self.theta = self.angular_vel * time_delta + self.theta
    self.pose_x = self.amcl_pose_x + self.linear_vel * time_delta * cos(self.angular_vel)
    self.pose_y = self.amcl_pose_y + self.linear_vel * time_delta * sin(self.angular_vel)
    print(self.pose_x, self.pose_y, self.theta)

  def euclidean_distance(self, goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.position.x - self.pose_x), 2) + pow((goal_pose.position.y - self.pose_y), 2))

  def set_linear_vel(self, goal_pose):
    """PD Controller for linear velocity"""
    self.prev_linear_vel_error = self.linear_vel_error
    self.linear_vel_error = self.euclidean_distance(goal_pose)
    linear_vel = self.linear_vel_Kp * self.linear_vel_error + \
                 self.linear_vel_Kd * (self.linear_vel_error - self.prev_linear_vel_error)

    # Flip the values bigger than 2048
    if linear_vel > 2048:
      linear_vel = 2048
    return linear_vel

  def steering_angle(self, goal_pose):
    """angle between current pose and the goal"""
    return atan2(goal_pose.position.y - self.pose_y, goal_pose.position.x - self.pose_x)

  def set_angular_vel(self, goal_pose):
    """PD Controller for angular velocity"""
    self.prev_angular_vel_error = self.angular_vel_error
    err = (self.theta - self.steering_angle(goal_pose))
    # Adjust value of steering angle which is outside -pi ~ pi
    if err > pi:
      err = err - 2 * pi
    elif err < -pi:
      err = err + 2 * pi
    self.angular_vel_error = err
    angular_vel = self.angular_vel_Kp * self.angular_vel_error + \
                  self.angular_vel_Kd * (self.angular_vel_error - self.prev_angular_vel_error) + 200

    # Flip the values outside the boundary of -2048 ~ 2048
    if angular_vel > 2048:
      angular_vel = 2048
    elif angular_vel < -2048:
      angular_vel = -2048
    return angular_vel

  def send_serial(self, steering_angle, gas_pedal):
    """Send serial a message to rally car to control"""
    if steering_angle > 0:
      sign = '+'
    else:
      sign = '-'
    serial_signal = "A{}{}+{}".format(sign, "%04d" % abs(steering_angle), "%04d" % gas_pedal)
    # construct and send the Ackermann steering commands to Car
    self.console_ser.write(serial_signal)

  def move2goal(self):
    """Moves the rally car to the goal."""
    print("Let's move!!")
    goal_pose = Pose()

    for i, way_point in enumerate(self.way_points):
      goal_pose.position.x = float(way_point[0])
      goal_pose.position.y = float(way_point[1])

      # update the waypoint if the distance between the goal is less than distance_tolerance.
      while self.euclidean_distance(goal_pose) >= self.distance_tolerance:
        # print(self.euclidean_distance(goal_pose))

        # interpolate current position
        self.interpolate_linear_vel()

        self.angular_vel = self.set_angular_vel(goal_pose)

        # send serial
        self.send_serial(self.angular_vel, self.set_linear_vel(goal_pose))

        # Publish at the desired rate.
        self.rate.sleep()
      print(way_point)

    # Stop rally car after getting gaol
    self.send_serial(self.bias, 0)

    # If we press control + C, the node will stop.
    rospy.spin()

  def shutdown(self):
    self.send_serial(2000, 0)


if __name__ == '__main__':
  try:
    # read the waypoint file
    with open('/home/crl4/ros_ws/src/myrallycar/scripts/way_point_speed_map.csv') as way_points:
      WAY_POINTS = list(csv.reader(way_points))

    # For the demo map
    # with open('/home/crl4/ros_ws/src/myrallycar/scripts/ui_auto_demo.txt') as way_points:
    #   WAY_POINTS = [i.strip().split(' ')[:2] for i in way_points.readlines()]

    print('load way_points')
    rally_car = RallyCar(way_points=WAY_POINTS)
    print('start')
    # rally_car.shutdown()
    rally_car.move2goal()

  except rospy.ROSInterruptException:
    pass
