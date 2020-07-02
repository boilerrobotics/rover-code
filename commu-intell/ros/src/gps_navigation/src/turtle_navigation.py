#!/usr/bin/env python3

"""
===============================================================================
Program Description 
	This program is for simulating PID navigation with turtlesim package.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        July 1, 2020
Status:         In progress
===============================================================================
"""

import rospy
from turtlesim.msg import Pose 

class TurtleNavigation:

    def __init__(self):
        rospy.init_node('turtle_navigation', anonymous=True)
        turtle_pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.turtle_pose_callback)
        self.one_sec_timer = rospy.Timer(rospy.Duration(1), self.one_sec_callback)
        self.turtle_pose = Pose()
        self.position = {
            'x': 0,
            'y': 0
        }
        rospy.spin()

    def one_sec_callback(self, timer_event):
        self.position['x'] = self.turtle_pose.x
        self.position['y'] = self.turtle_pose.y
        print(self.position)

    def turtle_pose_callback(self, turtle_pose):
        
        self.turtle_pose.x = turtle_pose.x
        self.turtle_pose.y = turtle_pose.y

        # print('x: {}'.format(self.turtle_pose.x))
        # print('y: {}'.format(self.turtle_pose.y))
        # print('---------------------------------------')



if __name__ == '__main__':
    try:
        turtle = TurtleNavigation()
    except rospy.ROSInterruptException:
        pass


