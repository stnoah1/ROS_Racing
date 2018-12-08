#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import Float32
import time


def kalman_filter(value):
  return value


def parse_serial(serial_data):
  return serial_data


def get_serial_data():
  # todo:update
  pub = rospy.Publisher('imu_info', Float32, queue_size=10)
  ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
  ser.close()
  ser.open()
  while ser.inWaiting() == 0:
    pass
  ser.write('IMU1')
  while True:
    serial_data = ser.readline().strip()
    print(serial_data)
    ang_vel = parse_serial(serial_data)
    filtered_ang_vel = kalman_filter(ang_vel)
    if filtered_ang_vel.startswith('I'):
      sign = filtered_ang_vel[1]
      value = int(filtered_ang_vel[2:7])
      if sign == '+':
        pub.publish(value)
      else:
        pub.publish(-1 * value)
    # ax


if __name__ == "__main__":
  print('test')
  rospy.init_node('imu_info', anonymous=True)
  get_serial_data()
