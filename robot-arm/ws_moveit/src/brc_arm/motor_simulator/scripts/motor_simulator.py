#!/usr/bin/env python
# include "ros/ros.h"
# include "roboclaw_node/MotorPosition.h"
# include <vector>
import rospy
from roboclaw_node.msg import MotorPosition, EncoderValues
from roboclaw_node.srv import HomeArm


def callback(data):
    global pub
    #rospy.logdebug(str(data))
    encoder_values = data.angle
    #pub.publish(encoder_values)


def handle_home_arm(req):
    status = [0] * len(req.motor_number)
    for i in range(0, len(req.motor_number)):
        rospy.loginfo("Homing joint: %d", i)
        status[i] = req.motor_number[i]
    return [status]


def listener():
    rospy.init_node('motor_simulator', anonymous=True)
    rospy.Subscriber("/brc_arm/motor_commands", MotorPosition, callback)
    # homeArm = rospy.Service('/brc_arm/home_arm', HomeArm, handle_home_arm)

    global pub
    pub = rospy.Publisher('/brc_arm/motor_positions',EncoderValues, queue_size=10)
    rate = rospy.Rate(10)

    rospy.spin()


if __name__ == '__main__':
    listener()