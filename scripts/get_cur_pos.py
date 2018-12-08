#!/usr/bin/env python

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
    self.amcl_time = rospy.Time.now()
    rospy.loginfo("Current AMCL pose: x=" + str(self.amcl_x) + "/ y=" + str(self.amcl_y) + "/ theta=" + str(self.theta))

  def estimate_pos(self):
    """todo: modify function to estimate the pos"""
    cur_pose = "{};{};{}".format(self.amcl_x, self.amcl_y, self.theta)

    return cur_pose

  def publish(self):
    print("publish")
    while not rospy.is_shutdown():
      cur_pose = self.estimate_pos()
      self.pub.publish(cur_pose)
      self.rate.sleep()
    rospy.spin()


if __name__ == "__main__":
  rospy.init_node("cur_pos", anonymous=True)
  cur_pose = CurrentPosition()
  cur_pose.publish()
