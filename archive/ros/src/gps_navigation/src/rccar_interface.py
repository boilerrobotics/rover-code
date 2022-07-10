#!/usr/bin/env python3

"""
===============================================================================
Program Description 
	This program execute command to RC car using Raspberry Pi and PCA9685.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        July 26, 2020
Status:         In progress
===============================================================================
"""

import rospy
from gps_navigation.msg import MoveCommand
import time

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

MAX_LEFT_ANGLE = 40
MAX_RIGHT_ANGLE = 140

class RCCar:
    def __init__(self):
        self.i2c_bus = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c_bus)
        self.pca.frequency = 60

        self.steering = servo.Servo(self.pca.channels[0])
        self.streering_angle = 90

        rospy.init_node('rccar_interface', anonymous=True)
        rospy.Subscriber('move_cmd', MoveCommand, self.message_callback)
        rospy.spin()


    def map_value(self, value, min_value, max_value):
        if value < -1:
            return min_value
        elif value > 1:
            return max_value
        else:
            slope = (max_value - min_value) / 2
            distance_from_min_x = value - (-1)
            return min_value + (slope * distance_from_min_x)

    def message_callback(self, payload):
        command_params = MoveCommand()
        command_params.speed = payload.speed
        command_params.streering = payload.streering

        self.streering_angle = self.map_value(command_params.streering, MAX_LEFT_ANGLE, MAX_RIGHT_ANGLE)
        self.streering_angle = round(self.streering_angle)
        self.steering.angle = self.streering_angle

        print('Speed: {}'.format(command_params.speed))
        print('Stearing Angle: {}'.format(self.streering_angle))
        print('---------------------------------------')
          

if __name__ == '__main__':
    car = RCCar()