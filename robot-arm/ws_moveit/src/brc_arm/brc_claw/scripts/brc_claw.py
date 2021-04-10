#!/usr/bin/env python
# include "ros/ros.h"
#include "roboclaw_node/MotorPosition.h"
#include <vector>
import rospy
from std_msgs.msg import Float32

def listener():
    rospy.init_node('brc_claw', anonymous=True)

    pub = rospy.Publisher('/brc_arm/claw_position', Float32, queue_size=10)
    rate = rospy.Rate(10)
    done = False
    while not rospy.is_shutdown() and not done:
        data = input("Input claw angle in degrees -> ")
        if("quit" in data):
            done = True
        else:
            angle = float(data)
            angle = angle * 3.14159 / 180.0
            pub.publish(angle)
        rate.sleep()

if __name__ == '__main__':
    listener()

