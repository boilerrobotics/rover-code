#!/usr/bin/env python3

"""
===============================================================================
Program Description 
	This program acts as remote controller to control RC car.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        July 27, 2020
Status:         In progress
===============================================================================
"""

import rospy
from gps_navigation.msg import MoveCommand

MAX_LEFT_ANGLE = 40
MAX_RIGHT_ANGLE = 140

current_angle = 90
current_speed = 0

def encode_cmd(raw_value, min_value, max_value):
    if raw_value > max_value:
        return 1
    elif raw_value < min_value:
        return -1
    else:
        slope = 2 / (max_value - min_value)
        distance_from_min_x = raw_value - min_value
        return (-1) + (distance_from_min_x * slope)

if __name__ == '__main__':
    movement_params = MoveCommand()
    pub = rospy.Publisher('move_cmd', MoveCommand, queue_size=10)
    rospy.init_node('manual_control', anonymous=True)
    while True:
        try:
            key = input('press q to exit: ')
            if key == 'q':
                exit()
            elif key == 'a':
                current_angle = current_angle - 5
                if current_angle < 40:
                    current_angle = 40
            elif key == 'd':
                current_angle = current_angle + 5
                if current_angle > 140:
                    current_angle = 140
            
            print('Angle: {}'.format(current_angle))
            print('Speed: {}'.format(current_speed))
            movement_params.streering = encode_cmd(current_angle, MAX_LEFT_ANGLE, MAX_RIGHT_ANGLE)
            movement_params.speed = current_speed
            pub.publish(movement_params)
            
        except KeyboardInterrupt:
            print()
            print("Ctrl C is detected")
            exit(0)
        except Exception as e:
            print(e)
            exit(0)