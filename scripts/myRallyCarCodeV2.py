#!/usr/bin/env python

import csv
import time
from math import pow, atan2, sqrt, cos, sin

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
    self.distance_tolerance = 1
    self.angular_vel_Kp = 1000
    self.angular_vel_Kd = 40
    self.linear_vel_Kp = 70
    self.linear_vel_Kd = 70

    self.bias = 430
    self.freq = 150

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

    # A subscriber to the topic 'cur_pos'. self.update_pose is called when a message of type Pose is received.
    self.pose_subscriber = rospy.Subscriber('/cur_pos', String, self.update_pose)

    # A subscriber to the topic 'imu_info'. self.update_angular_vel is called when a message of type Pose is received.
    # self.imu_info_subscriber = rospy.Subscriber('/imu_info', Float32, self.update_angular_vel)

    # Initialize steering
    self.send_serial(self.bias, 0)
    time.sleep(1)

  def update_pose(self, data):
    """Callback function which is called when a new message of type Pose is
    received by the subscriber."""
    self.prev_amcl_pose_x = self.amcl_pose_x
    self.prev_amcl_pose_y = self.amcl_pose_y
    self.prev_amcl_theta = self.amcl_theta
    self.prev_amcl_stamp = self.amcl_stamp

    pose = data.data.split(";")
    self.amcl_pose_x = round(float(pose[0]), 4)
    self.amcl_pose_y = round(float(pose[1]), 4)
    self.amcl_theta = round(float(pose[2]), 4)

    self.pose_x = self.amcl_pose_x
    self.pose_y = self.amcl_pose_y
    self.theta = self.amcl_theta
    self.amcl_stamp = time.time()

    time_delta = self.amcl_stamp - self.prev_amcl_stamp
    distant_delta = sqrt(pow((self.pose_x - self.prev_amcl_pose_x), 2) + pow((self.pose_y - self.prev_amcl_pose_y), 2))
    theta_delta = self.theta - self.prev_amcl_theta

    self.estimate_linear_vel = round(distant_delta / time_delta, 4)
    self.estimate_angular_vel = round(theta_delta / time_delta, 4)

    self.linear_vel = self.estimate_linear_vel
    self.angular_vel = self.estimate_angular_vel

    # rospy.loginfo("Current AMCL pose: x=" + str(self.pos_x) + "/ y=" + str(self.pos_y) + "/ theta=" + str(self.theta))

  def update_angular_vel(self, data):
    self.angular_vel = self.angular_vel + data.data * (time.time() - self.imu_stamp)
    self.imu_stamp = time.time()

  def interpolate_linear_vel(self):
    time_delta = time.time() - self.amcl_stamp
    self.theta = self.angular_vel * time_delta + self.theta
    self.pose_x = self.amcl_pose_x + self.linear_vel * time_delta * cos(self.theta)
    self.pose_y = self.amcl_pose_y + self.linear_vel * time_delta * sin(self.theta)

  def euclidean_distance(self, goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.position.x - self.pose_x), 2) + pow((goal_pose.position.y - self.pose_y), 2))

  def set_linear_vel(self, goal_pose):
    """PD Controller for linear velocity"""
    self.prev_linear_vel_error = self.linear_vel_error
    self.linear_vel_error = self.euclidean_distance(goal_pose)
    return self.linear_vel_Kp * self.linear_vel_error + \
           self.linear_vel_Kd * (self.linear_vel_error - self.prev_linear_vel_error)

  def steering_angle(self, goal_pose):
    """angle between current pose and the goal"""
    return atan2(goal_pose.position.y - self.pose_y, goal_pose.position.x - self.pose_x)

  def set_angular_vel(self, goal_pose):
    """PD Controller for angular velocity"""
    self.prev_angular_vel_error = self.angular_vel_error
    self.angular_vel_error = (self.theta - self.steering_angle(goal_pose))
    return self.angular_vel_Kp * self.angular_vel_error + \
           self.angular_vel_Kd * (self.angular_vel_error - self.prev_angular_vel_error)

  def send_serial(self, steering_angle, gas_pedal):
    if steering_angle > 0:
      sign = '+'
    else:
      sign = '-'
    serial_signal = "A{}{}+{}".format(sign, "%04d" % abs(steering_angle), "%04d" % gas_pedal)
    print(serial_signal)
    # construct and send the Ackermann steering commands to Car
    console_ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
    console_ser.write(serial_signal)

  def move2goal(self):
    """Moves the rally car to the goal."""
    print("Let's move!!")
    goal_pose = Pose()

    for i, way_point in enumerate(self.way_points):
      goal_pose.position.x = way_point[0]
      goal_pose.position.y = way_point[1]

      while self.euclidean_distance(goal_pose) >= self.distance_tolerance:
        # print(self.euclidean_distance(goal_pose))

        # interpolate current position
        # self.interpolate_linear_vel()

        # send serial
        self.send_serial(self.set_angular_vel(goal_pose), self.set_linear_vel(goal_pose))

        # Publish at the desired rate.
        self.rate.sleep()
      print("update point {}!!!!!!!!!!!!!!".format(i))

    # Stop rally car after getting gaol
    self.send_serial(0, 0)

    # If we press control + C, the node will stop.
    rospy.spin()

  def shutdown(self):
    self.send_serial(self.bias, 0)


if __name__ == '__main__':
  try:
    with open('way_point_speed_map.csv', newline='') as way_points:
      WAY_POINTS = list(csv.reader(way_points))
    print('load way_points')
    rally_car = RallyCar(way_points=WAY_POINTS)
    print('start')
    # rally_car.shutdown()
    rally_car.move2goal()

  except rospy.ROSInterruptException:
    pass
