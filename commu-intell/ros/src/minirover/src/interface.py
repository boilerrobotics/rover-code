#!/usr/bin/env python3

"""
===============================================================================
Program Description 
	This program recieve the command from joy stick or navigation algorithm then
    process into driving command to the mini-rover.
Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        November 9, 2020
Status:         In progress
===============================================================================
"""

import rospy
from sensor_msgs.msg import Joy
from minirover.msg import WheelSpeed

class WheelInterface:

    def __init__(self):
        self.pub = rospy.Publisher('wheel_speed', WheelSpeed, queue_size=1)
        rospy.init_node('minirover_interface', anonymous=True)
        rospy.Subscriber('joy', Joy, self.callback)
        
        rospy.spin()

    def callback(self, payload):
        
        speed = WheelSpeed()
        WheelSpeed.left = payload.axes[1]
        WheelSpeed.right = payload.axes[4]
        self.pub.publish(speed)

if __name__ == '__main__':
    
    interface = WheelInterface()