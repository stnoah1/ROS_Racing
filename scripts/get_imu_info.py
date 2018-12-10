#!/usr/bin/env python

## Course: ECET581 Fall2018
## Authors: Hyung-gun Chi and Donghun Lee
## This file publishes the topic 'imu_info' after read the serial data from the serial port


import serial
import rospy
from std_msgs.msg import Float32

G = 9.807


def kalman_filter(value):
  "https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050"
  return value


def parse_serial(ang_vel):
  """Parse imu data with the unit of m/s^2"""
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
  ser.write('IMU0')
  while True:
    serial_data = ser.readline().strip()
    if serial_data.startswith('I'):
      filtered_ang_vel = kalman_filter(serial_data)
      ax = parse_serial(filtered_ang_vel)
      pub.publish(ax)
    else:
      print(serial_data)


if __name__ == "__main__":
  rospy.init_node('imu_info', anonymous=True)
  get_serial_data()
