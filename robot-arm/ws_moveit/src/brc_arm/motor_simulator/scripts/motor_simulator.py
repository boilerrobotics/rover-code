#!/usr/bin/env python
# include "ros/ros.h"
# include "roboclaw_node/MotorPosition.h"
# include <vector>
from tkinter import CallWrapper
import rospy
from roboclaw_node.msg import MotorPosition, EncoderValues
from roboclaw_node.srv import HomeArm, MoveClaw


def callback_motor(data):
    global pub
    global claw
    rospy.logdebug(str(data))
    encoder_values = list(data.angle)
    if claw == 0: # Hardcoded rlink1
        encoder_values[6] = 0.175
    elif claw == 1: # Hardcoded rlink1
        encoder_values[6] = 0.5
    pub.publish(encoder_values)


def handle_home_arm(req):
    status = [0] * len(req.to_home)
    for i in range(0, len(req.to_home)):
        rospy.loginfo("Homing joint: %d", i)
        status[i] = req.to_home[i]
    return [status]


def handle_move_claw(req):
    global claw
    if req.goal_state == 1:
        claw = 1
    elif req.goal_state == 0:
        claw = 0
    return [claw]


def listener():
    rospy.init_node('motor_simulator', anonymous=True)
    rospy.Subscriber("/brc_arm/motor_commands", MotorPosition, callback_motor)

    global pub
    pub = rospy.Publisher('/brc_arm/motor_positions', EncoderValues, queue_size=10)

    global claw
    claw = 0

    homeArm = rospy.Service('/brc_arm/home_arm', HomeArm, handle_home_arm)
    moveClaw = rospy.Service('/brc_arm/move_claw', MoveClaw, handle_move_claw)
    rate = rospy.Rate(10)


    rospy.spin()


if __name__ == '__main__':
    listener()
