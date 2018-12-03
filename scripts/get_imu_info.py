import serial
import rospy
from std_msgs.msg import Float32


def kalman_filter(value):
  return value


def parse_serial(serial_data):
  return serial_data


def get_serial_data():
  # todo:update
  pub = rospy.Publisher('imu_info', Float32, queue_size=10)
  flag = True
  with serial.Serial("/dev/ttyACM0", baudrate=115200) as ser:
    if flag:
      ser.write('IMU1')
      flag = False
    # "I%+05d%+05d%+05dU"
    serial_data = ser.readline()
    ang_vel = parse_serial(serial_data)
    filtered_ang_vel = kalman_filter(ang_vel)
    pub.publish(filtered_ang_vel)


if __name__ == "__main__":
  rospy.init_node('imu_info', anonymous=True)
  get_serial_data()
