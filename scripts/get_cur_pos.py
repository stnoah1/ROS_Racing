#!/usr/bin/env python

## Course: ECET581 Fall2018
## Authors: Hyung-gun Chi and Donghun Lee
## This file publishes the topic 'cur_pos' when it gets the amcl position from the topic 'amcl_pose'

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class CurrentPosition:
  def __init__(self):
    self.pub = rospy.Publisher('cur_pos', String, queue_size=10)
    self.amcl_x = None
    self.amcl_y = None
    self.theta = None
    self.rate = rospy.Rate(150)  # 150hz
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)

  def callback_pose(self, data):
    """Callback for the topic subscriber. Prints the current received data on the topic."""
    self.amcl_x = data.pose.pose.position.x
    self.amcl_y = data.pose.pose.position.y
    orientationQ = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                    data.pose.pose.orientation.w]

    euler = euler_from_quaternion(orientationQ)
    self.theta = euler[2]
    if not rospy.is_shutdown():
      # rospy.loginfo("Current AMCL pose: x=" + str(self.amcl_x) + "/ y=" + str(self.amcl_y) + "/ theta=" + str(self.theta))
      # publish x, y, orientation
      cur_pose = "{};{};{}".format(self.amcl_x, self.amcl_y, self.theta)
      self.pub.publish(cur_pose)

  def run(self):
    rospy.spin()


if __name__ == "__main__":
  rospy.init_node("cur_pos", anonymous=True)
  cur_pose = CurrentPosition()
  cur_pose.run()
