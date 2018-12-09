#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import Float32

G = 9.807


def kalman_filter(value):
  "https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050"
  return value


def parse_serial(ang_vel):
  sign = ang_vel[1]
  value = int(ang_vel[2:7]) / (2 * G)
  if sign == '+':
    return value
  else:
    return -1 * value


def get_serial_data():
  pub = rospy.Publisher('imu_info', Float32, queue_size=10)
  ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
  ser.close()
  ser.open()
  while ser.inWaiting() == 0:
    pass
  ser.write('IMU1')
  while True:
    serial_data = ser.readline().strip()
    if serial_data.startswith('I'):
      filtered_ang_vel = kalman_filter(serial_data)
      ax = parse_serial(filtered_ang_vel)
      pub.publish(ax)


if __name__ == "__main__":
  print('test')
  rospy.init_node('imu_info', anonymous=True)
  get_serial_data()
