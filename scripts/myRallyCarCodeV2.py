#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import serial


class RallyCar:

  def __init__(self, way_points):
    # Creates a node with name 'rallycar_controller' and make sure it is a unique node (using anonymous=True).
    rospy.init_node('rallycar_controller', anonymous=True)

    # A subscriber to the topic 'cur_pos'. self.update_pose is called
    # when a message of type Pose is received.
    self.pose_subscriber = rospy.Subscriber('cur_pos', Pose, self.update_pose)

    self.pose = Pose()
    self.rate = rospy.Rate(150)
    self.distance_tolerance = 0.01
    self.way_points = way_points

  def update_pose(self, data):
    """Callback function which is called when a new message of type Pose is
    received by the subscriber."""
    self.pose = data
    self.pose.x = round(self.pose.x, 4)
    self.pose.y = round(self.pose.y, 4)

  def euclidean_distance(self, goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                pow((goal_pose.y - self.pose.y), 2))

  def linear_vel(self, goal_pose, Kp=6, Kd=1):
    """PD Controller for linear velocity"""
    return Kp * self.euclidean_distance(goal_pose)

  def steering_angle(self, goal_pose):
    """angle between current pose and the goal"""
    return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

  def angular_vel(self, goal_pose, Kp=6, Kd=1):
    """PD Controller for angular velocity"""
    return Kp * (self.steering_angle(goal_pose) - self.pose.theta)

  def send_serial(self, steering_angle, gas_pedal):
    serial_signal = "A+{}+{}".format("%04d" % steering_angle, "%04d" % gas_pedal)
    console_ser = serial.Serial("/dev/ttyACM0", baudrate=115200)

    # construct and send the Ackermann steering commands to Car
    console_ser.write(serial_signal)

  def move2goal(self):
    """Moves the turtle to the goal."""
    goal_pose = Pose()

    for way_point in self.way_points:
      goal_pose.x = way_point[0]
      goal_pose.y = way_point[1]

      while self.euclidean_distance(goal_pose) >= self.distance_tolerance:
        # Publishing our vel_msg
        self.send_serial(self.linear_vel(goal_pose), self.angular_vel(goal_pose))

        # Publish at the desired rate.
        self.rate.sleep()

    # Stop rally car after getting gaol
    self.send_serial(0, 0)

    # If we press control + C, the node will stop.
    rospy.spin()


if __name__ == '__main__':
  try:
    WAY_POINTS = [
      [1.00882411003, -0.110736370087],
      [4.99138975143, -0.115155220032],
      [9.00156021118, -0.103336811066],
      [12.9005899429, -0.0280865430832],
      [18.3351726532, 0.169030070305],
      [23.3436813354, 0.330986976624],
      [29.8973789215, 0.398444473743]
    ]
    rally_car = RallyCar(way_points=WAY_POINTS)
    rally_car.move2goal()
  except rospy.ROSInterruptException:
    pass
