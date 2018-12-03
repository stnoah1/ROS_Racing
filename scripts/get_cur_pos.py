import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from math import atan2


class CurrentPosition:
  def __init__(self):
    self.pub = rospy.Publisher('cur_pos', Pose, queue_size=10)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)
    self.amcl_prev_x = None
    self.amcl_prev_y = None

  def get_angle(self):
    if not self.amcl_prev_x:
      theta = atan2(self.amcl_y - self.amcl_prev_y, self.amcl_x - self.amcl_prev_x)
    else:
      theta = 0
    return theta

  def callback_pose(self, data):
    """Callback for the topic subscriber. Prints the current received data on the topic."""
    self.amcl_prev_x = self.amcl_x
    self.amcl_prev_y = self.amcl_y
    self.amcl_x = data.pose.pose.position.x
    self.amcl_y = data.pose.pose.position.y
    self.amcl_time = rospy.Time.now()
    self.theta = self.get_angle()
    rospy.loginfo("Current AMCL pose: x=" + str(self.amcl_x) + "y=" + str(self.amcl_y) + "theta=" + str(self.theta))

  def estimate_pos(self):
    """todo: modify function to estimate the pos"""

    cur_pose = Pose()
    cur_pose.x = self.amcl_x
    curr_pos.y = self.amcl_y
    curr_pos.theta = self.theta
    return curr_pos

  def publish(self):
    rate = rospy.Rate(150)  # 150hz
    while not rospy.is_shutdown():
      cur_pose = self.estimate_pos()
      self.pub.publish(cur_pose)
      rate.sleep()
    rospy.spin()


if __name__ == "__main__":
  rospy.init_node("cur_pos", anonymous=True)
  curr_pos = CurrentPosition()
  curr_pos.publish()
