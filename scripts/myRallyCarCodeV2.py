#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from math import pow, atan2, sqrt
import time
import serial


class RallyCar:
  def __init__(self, way_points):
    # Creates a node with name 'rallycar_controller' and make sure it is a unique node (using anonymous=True).
    print('initialize param')
    rospy.init_node('rallycar_controller', anonymous=True)

    # initialize parameters
    self.bias = 430
    self.pos_x = 0
    self.pos_y = 0
    self.theta = 0
    self.rate = rospy.Rate(150)
    self.way_points = way_points
    self.prev_angular_vel_error = 0
    self.angular_vel_error = 0
    self.prev_linear_vel_error = 0
    self.linear_vel_error = 0

    # parameters to set up
    self.distance_tolerance = 1
    self.angular_vel_Kp = 1000
    self.angular_vel_Kd = 40
    self.linear_vel_Kp = 70
    self.linear_vel_Kd = 70

    # A subscriber to the topic 'cur_pos'. self.update_pose is called
    # when a message of type Pose is received.
    self.pose_subscriber = rospy.Subscriber('/cur_pos', String, self.update_pose)

    # self.console_ser.close()
    # self.console_ser.open()
    self.send_serial(self.bias, 0)
    time.sleep(3)

  def update_pose(self, data):
    """Callback function which is called when a new message of type Pose is
    received by the subscriber."""
    pos = data.data.split(";")
    self.pos_x = round(float(pos[0]), 4)
    self.pos_y = round(float(pos[1]), 4)
    self.theta = round(float(pos[2]), 4)
    # rospy.loginfo("Current AMCL pose: x=" + str(self.pos_x) + "/ y=" + str(self.pos_y) + "/ theta=" + str(self.theta))

  def euclidean_distance(self, goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.position.x - self.pos_x), 2) + pow((goal_pose.position.y - self.pos_y), 2))

  def linear_vel(self, goal_pose):
    """PD Controller for linear velocity"""
    self.prev_linear_vel_error = self.linear_vel_error
    self.linear_vel_error = self.euclidean_distance(goal_pose)
    return self.linear_vel_Kp * self.linear_vel_error + \
           self.linear_vel_Kd * (self.linear_vel_error - self.prev_linear_vel_error)

  def steering_angle(self, goal_pose):
    """angle between current pose and the goal"""
    return atan2(goal_pose.position.y - self.pos_y, goal_pose.position.x - self.pos_x)

  def angular_vel(self, goal_pose):
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
    # todo: update
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
        # send serial
        linear_vel = self.linear_vel(goal_pose)
        angular_vel = self.angular_vel(goal_pose)
        self.send_serial(self.angular_vel(goal_pose), self.linear_vel(goal_pose))

        # Publish at the desired rate.
        self.rate.sleep()
      print("update point {}!!!!!!!!!!!!!!".format(i))

    # Stop rally car after getting gaol
    self.send_serial(0, 0)

    # If we press control + C, the node will stop.
    rospy.spin()

  def shutdown(self):
    self.send_serial(2000, 200)


if __name__ == '__main__':
  try:
    WAY_POINTS = [
      (2.27361798286, 1.04696655273),
      (2.49206614494, 3.555751323),
      (2.47692918777, 5.60992193222),
      (2.47692918777, 7.60992193222),
      (2.57692918777, 9.60992193222),
      (2.67692918777, 11.60992193222),
      (2.77692918777, 13.60992193222),
      (2.87692918777, 15.60992193222),
      (2.97692918777, 17.60992193222),
      (3.17692918777, 19.60992193222),
      (3.27692918777, 21.60992193222),
      (3.27692918777, 23.60992193222),
      (3.27692918777, 25.60992193222),
      (3.27692918777, 27.60992193222),
      (3.27692918777, 29.60992193222),
      (3.27692918777, 31.60992193222),
      (3.27692918777, 33.60992193222),
      (3.27692918777, 35.60992193222),
      (3.27692918777, 37.60992193222),
      (-1.39152431488, 38.3741378784),
      (-3.39152431488, 38.3741378784),
      (-5.39152431488, 38.3741378784),
      # (2.57075452805, 11.1395378113),
      # (2.80263257027, 16.9178562164),
      # (2.94566726685, 21.8589191437),
      # (2.91902089119, 26.6184883118),
      # (3.08603787422, 33.2159004211),
      (2.95427036285, 38.4813156128),
      (-1.39152431488, 38.3741378784),
    ]
    print('start')
    rally_car = RallyCar(way_points=WAY_POINTS)
    # rally_car.shutdown()
    rally_car.move2goal()

  except rospy.ROSInterruptException:
    pass
