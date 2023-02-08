#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

SerialPort = '/dev/ttyUSB0'

def imu_interface():
    pub = rospy.Publisher('imu', String, queue_size=10)
    rospy.init_node('imu_interface', anonymous=True)
    try:
        serial_port = serial.Serial(SerialPort, 9600)
    except Exception as e:
        print(e)
        exit(1)
    while not rospy.is_shutdown():
        message = serial_port.read()
        rospy.loginfo(message)
        pub.publish(message)

if __name__ == '__main__':
    try:
        imu_interface()
    except rospy.ROSInterruptException:
        pass
