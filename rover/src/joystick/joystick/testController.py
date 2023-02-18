import sys

import pygame
from pygame.locals import *

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class MyPublisher(Node):

    def __init__(self):
        super().__init__("joystick")
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timerPeriod = .1
        self.timer = self.create_timer(timerPeriod, self.timerCallback)
        self.leftInputs = Vector3()
        self.rightInputs = Vector3()


    def timerCallback(self):
        msg = Twist()

        for event in pygame.event.get():

            if event.type == JOYAXISMOTION:
                if event.axis == 1:
                    if abs(event.value) > 0.05:
                        if abs(event.value) > 1:
                            self.leftInputs.x = float(round(event.value))
                        else:
                            self.leftInputs.x = event.value 
                    else: 
                        self.leftInputs.x = 0.0
                if event.axis == 3:
                    if abs(event.value) > 0.05:
                        if abs(event.value) > 1:
                            self.rightInputs.z = float(round(event.value))
                        else:
                            self.rightInputs.z = event.value 
                    else: 
                        self.rightInputs.z = 0.0

        msg.linear = self.leftInputs
        msg.angular = self.rightInputs
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: Linear: {msg.linear.x} Angular: {msg.angular.z}'
        )

pygame.init() 

pygame.joystick.init()

joysticks = [
    pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())
]

for joystick in joysticks:

    print(joystick.get_name())

 

def main(args = None):
    rclpy.init(args = args)

    myPublisher = MyPublisher()
    
    rclpy.spin(myPublisher)

    myPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
