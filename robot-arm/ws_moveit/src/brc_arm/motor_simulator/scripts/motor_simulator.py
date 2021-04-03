#!/usr/bin/env python
# include "ros/ros.h"
#include "roboclaw_node/MotorPosition.h"
#include <vector>
import rospy
from roboclaw_node.msg import MotorPosition, EncoderValues

def callback(data):
    global pub
    #rospy.loginfo(str(data))

    encoder_values = data.angle
    pub.publish(encoder_values)


def listener():
    rospy.init_node('motor_simulator', anonymous=True)
    rospy.Subscriber("/brc_arm/motor_commands", MotorPosition, callback)

    global pub
    pub = rospy.Publisher('/brc_arm/motor_positions', EncoderValues, queue_size=10)
    rate = rospy.Rate(10)

    rospy.spin()

if __name__ == '__main__':
    listener()

