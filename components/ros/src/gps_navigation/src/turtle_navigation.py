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
from geometry_msgs.msg import Twist
from math import pow, sqrt, atan2, degrees


class Location:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class TurtleNavigation:

    def __init__(self):
        rospy.init_node('turtle_navigation', anonymous=True)
        rospy.Subscriber('/turtle1/pose', Pose, self.turtle_pose_callback)
        self.cmd_pub = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        self.one_sec_timer = rospy.Timer(
            rospy.Duration(1), self.one_sec_callback)
        self.turtle_pose = Pose()
        self.cmd = Twist()
        self.position = Location()
        self.target = Location()
        self.distance_tolerate = 0.1
        self.mute_flag = True

        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        self.pi = 2
        self.start_flag = False

    def get_target(self):
        self.mute_flag = True
        coordinate_input = input('Please enter X target position (0-10): ')
        try:
            coordinate_input = int(coordinate_input)
            if coordinate_input < 0 or coordinate_input > 10:
                print('Invalid input. New coordinate will not be set.')
            else:
                self.target.x = coordinate_input
        except Exception as e:
            print(e)

        coordinate_input = input('Please enter Y target position (0-10): ')
        try:
            coordinate_input = int(coordinate_input)
            if coordinate_input < 0 or coordinate_input > 10:
                print('Invalid input. New coordinate will not be set.')
            else:
                self.target.y = coordinate_input
        except Exception as e:
            print(e)
        self.mute_flag = False
        self.go_to_target()
        # rospy.spin()

    def go_to_target(self):
        self.start_flag = True
        while self.distance > self.distance_tolerate:
            if self.distance > self.distance_tolerate*30:
                self.cmd.linear.x = 1
            else:
                self.cmd.linear.x = 0.1


    def one_sec_callback(self, timer_event):
        self.target_heading = self.calulate_heading(
            self.turtle_pose, self.target)
        self.current_heading = self.calulate_heading(
            self.position, self.turtle_pose)
        self.position.x = self.turtle_pose.x
        self.position.y = self.turtle_pose.y
        self.calculate_distance()
        if not self.mute_flag:
            print('Distance from target: {}'.format(self.distance))
            print('Heading to target: {}'.format(self.target_heading))
            print('Current heading:{}'.format(self.current_heading))

        dif_heading = self.target_heading - self.current_heading
        steering = dif_heading/180 * self.pi
        self.cmd.angular.z = round(steering, 3)

        if self.start_flag:
            self.cmd_pub.publish(self.cmd)

    def turtle_pose_callback(self, turtle_pose):
        self.turtle_pose.x = turtle_pose.x
        self.turtle_pose.y = turtle_pose.y

        # print('x: {}'.format(self.turtle_pose.x))
        # print('y: {}'.format(self.turtle_pose.y))
        # print('---------------------------------------')

    def calculate_distance(self):
        self.distance = round(sqrt(pow(self.target.x - self.position.x, 2) +
                             pow(self.target.y - self.position.y, 2)), 3)

    def calulate_heading(self, loc_a, loc_b):
        dif_x = loc_b.x - loc_a.x
        dif_y = loc_b.y - loc_a.y

        heading = degrees(atan2(dif_y, dif_x))

        return heading


if __name__ == '__main__':
    try:
        turtle = TurtleNavigation()
        turtle.get_target()

    except rospy.ROSInterruptException:
        pass
