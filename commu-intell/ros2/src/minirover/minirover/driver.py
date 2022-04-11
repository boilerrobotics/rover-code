'''
===============================================================================
Program Description 
	The driver program for minirover.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         March 21, 2022
Version:        0.1.0
===============================================================================
'''

import rclpy
import busio
from rclpy.node import Node
from geometry_msgs.msg import Twist
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

class Motor:
    '''
    Class for DC motor that connect to PCA9685
    '''

    MAX_SCALE = (2**16) - 1 
    SPEED_RATIO = 0.6
    
    def __init__(self, pca, forward, backward):
        '''
            Need PWM channals for forward and backward and pca instance
        '''
        self.forward = pca.channels[forward]
        self.backward = pca.channels[backward]

    def set_speed(self, speed):
        '''
            Pass -1.0 to 1.0 for speed
        '''
        if -1 <= speed < 0:
            self.forward.duty_cycle = 0
            self.backward.duty_cycle = int(-1 * speed * self.MAX_SCALE * self.SPEED_RATIO)
        
        elif 0 < speed <= 1:
            self.forward.duty_cycle = int(speed * self.MAX_SCALE * self.SPEED_RATIO)
            self.backward.duty_cycle = 0
        
        else:
            self.forward.duty_cycle = 0
            self.backward.duty_cycle = 0

class MiniRover(Node):

    def __init__(self) -> None:
        super().__init__('minirover')
        # ros topic for command from teleop keyboard package
        self.cmd = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10)
        self.cmd  # prevent unused variable warning

        # motor objects
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

    def clamp(self, num, min_value, max_value):
        return max(min(num, max_value), min_value)

    def cmd_callback(self, cmd):

        speed_diff = cmd.angular.z / 2

        left_speed = self.clamp(cmd.linear.x - speed_diff, -1, 1)
        right_speed = self.clamp(cmd.linear.x + speed_diff, -1, 1)

        self.get_logger().info(f'Left Speed: {left_speed} Right Speed: {right_speed}')

        for wheel in self.left_wheels:
            wheel.set_speed(left_speed)

        for wheel in self.right_wheels:
            wheel.set_speed(right_speed)


def main(args=None):
    rclpy.init(args=args)
    
    rover = MiniRover()
    rclpy.spin(rover)
    rover.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
