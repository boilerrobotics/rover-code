import sys

import pygame
from pygame.locals import *

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Float64MultiArray


class MyPublisher(Node):

    def __init__(self):
        super().__init__("myPublisher")
        self.publisher_ = self.create_publisher(Float64MultiArray, 'demo', 10)
        timerPeriod = .1
        self.timer = self.create_timer(timerPeriod, self.timerCallback)
        self.joyInputs = [0.0, 0.0]
    
    def timerCallback(self):
        msg = Float64MultiArray()


        for event in pygame.event.get():

            # msg.data = str(event)
            # self.publisher_.publish(msg)
            # self.get_logger().info(f'Publishing: "{msg.data}"')

            # if event.type == JOYBUTTONDOWN:

            #     msg.data = str(event)
            #     self.publisher_.publish(msg)
            #     self.get_logger().info(f'Publishing: "{msg.data}"')

            # if event.type == JOYBUTTONUP:

            #     msg.data = str(event)
            #     self.publisher_.publish(msg)
            #     self.get_logger().info(f'Publishing: "{msg.data}"')

            if event.type == JOYAXISMOTION:
                # if event.axis == 0:
                #     if abs(event.value) > 0.1:
                #         joyInputs[0] = event.value 
                #     else: 
                #         joyInputs[0] = 0
                # if event.axis == 3:
                #     if abs(event.value) > 0.1:
                #         joyInputs[1] = event.value 
                #     else: 
                #         joyInputs[1] = 0
                if event.axis == 1:
                    if abs(event.value) > 0.05:
                        if abs(event.value) > 1:
                            self.joyInputs[0] = float(round(-event.value))
                        else:
                            self.joyInputs[0] = -event.value 
                    else: 
                        self.joyInputs[0] = 0.0
                if event.axis == 4:
                    if abs(event.value) > 0.05:
                        if abs(event.value) > 1:
                            self.joyInputs[1] = float(round(-event.value))
                        else:
                            self.joyInputs[1] = -event.value 
                    else: 
                        self.joyInputs[1] = 0.0

        msg.data = self.joyInputs
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


            # if event.type == JOYHATMOTION:

            #     msg.data = str(event)
            #     self.publisher_.publish(msg)
            #     self.get_logger().info(f'Publishing: "{msg.data}"')


pygame.init() 

pygame.joystick.init()

joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]

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
