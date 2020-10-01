#!/usr/bin/env python3

"""
===============================================================================
Program Description 
	This program recieve the command from joy stick and drive the minirover.
Author:         Thirawat Bureetes(Thirawat.tam@gmail.com), Ben Sukboontip, Pum Khai, Bryan Yakimisky, Preston Rahim
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Version:        September 30, 2020
Status:         In progress
===============================================================================
"""

import rospy
from sensor_msgs.msg import Joy
from board import SCL, SDA
import busio
import adafruit_pca9685

class Motor:
    '''
    Class for DC motor that connect to PCA9685
    '''

    MAX_SCALE = (2**16) - 1 
    
    def __init__(self, pca, forward, backward):
        '''
            Need PWM channals for forward and backward and pca instance
        '''
        self.forward = pca.channel[forward]
        self.backward = pca.channel[backward]

    def set_speed(self, speed):
        '''
            Pass -1.0 to 1.0 for speed
        '''
        if -1 <= speed < 0:
            self.forward.duty_cycle = 0
            self.backward.duty_cycle = speed * self.MAX_SCALE
        
        elif 0 < speed <= 1:
            self.forward.duty_cycle = speed * self.MAX_SCALE
            self.backward.duty_cycle = 0
        
        else:
            self.forward.duty_cycle = 0
            self.backward.duty_cycle = 0

class Minirover:

    def __init__(self):
        self.i2c_bus = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c_bus)
        self.pca.frequency = 200

        self.left_wheels = [
            Motor(self.pca, 4, 5),
            Motor(self.pca, 8, 9),
            Motor(self.pca, 12, 13)
        ]

        self.right_wheels = [
            Motor(self.pca, 6, 7),
            Motor(self.pca, 10, 11),
            Motor(self.pca, 14, 15)
        ]

        rospy.init_node('minirover_driver', anonymous=True)
        rospy.Subscriber('joy', Joy, callback)
        rospy.spin()

    def callback(self, payload):
        
        params = Joy()
        print(payload)

        # Need to extract Joy information
        left_speed = 0
        right_speed = 0

        for wheel in self.left_wheels:
            wheel.set_speed(left_speed)

        for wheel in self.right_wheels:
            wheel.set_speed(left_speed)

if __name__ == '__main__':
    
    rover = Minirover()