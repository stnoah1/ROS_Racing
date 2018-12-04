#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import Float32


def kalman_filter(value):
  # todo : complete
  "https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050"
  return value


def parse_serial(serial_data):
  if serial_data[1] == '+':
    return int(serial_data[2:7])
  else:
    return -1 * int(serial_data[2:7])


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
    print(serial_data)
    if serial_data.startswith("I"):
      ang_vel = parse_serial(serial_data)
      filtered_ang_vel = kalman_filter(ang_vel)
      pub.publish(filtered_ang_vel)


if __name__ == "__main__":
  rospy.init_node('imu_info', anonymous=True)
  get_serial_data()
