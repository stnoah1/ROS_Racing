import serial
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Point
import math


class RallyCar:
  def __init__(self, way_points):
    rospy.Subscriber("cur_pos", Point, self.get_cur_pos)
    self.cur_x = 0
    self.cur_y = 0
    self.thread_hold = 1
    self.rate = rospy.Rate(150)
    self.way_points = way_points
    self.way_point_idx = 0
    self.way_point = self.way_points[self.way_point_idx]
    self.dist = None

  def dist_to_goal(self):
    return math.sqrt((self.cur_x - self.way_point[0]) ** 2 + (self.cur_y - self.way_point[1]) ** 2)

  def update_way_point(self):
    if self.dist_to_goal() < self.thread_hold:
      self.way_point_idx = 0
      self.way_point = self.way_points[self.way_point_idx]

  def drive_car(self):
    while not rospy.is_shutdown():
      self.update_way_point()
      self.send_serial()
      self.rate.sleep()

  def get_cur_pos(self, data):
    self.cur_x = data.x
    self.cur_y = data.y

  def get_velocity(self):
    k_p = 10
    k_d = 80
    v_target = 200
    error = v_target - speed
    speed = k_p * error + k_d * (error - prev_error)
    return speed, error

  def get_steering_angle(self):
    k_p = 10
    k_d = 80
    error = angle - prev_angle
    control_angle = k_p * error + k_d * (error - prev_error)
    angle = angle + control_angle

    return angle, error

  def send_serial(self, steering_angle, gas_pedal):
    # todo: consider sign
    serial_signal = "A+{}+{}".format("%04d" % steering_angle, "%04d" % gas_pedal)
    console_ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
    # construct and send the Ackermann steering commands to Car
    console_ser.write(serial_signal)

  def run(self):
    rospy.spin()


if __name__ == "__main__":
  rospy.init_node('drive_rally_car', anonymous=True)
  way_points = [
    [1.00882411003, -0.110736370087],
    [4.99138975143, -0.115155220032],
    [9.00156021118, -0.103336811066],
    [12.9005899429, -0.0280865430832],
    [18.3351726532, 0.169030070305],
    [23.3436813354, 0.330986976624],
    [29.8973789215, 0.398444473743]
  ]
  rally_car = RallyCar(way_points=way_points)
  rally_car.run()
