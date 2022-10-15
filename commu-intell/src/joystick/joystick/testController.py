import sys

import pygame
from pygame.locals import *

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MyPublisher(Node):

    def __init__(self):
        super().__init__("myPublisher")
        self.publisher_ = self.create_publisher(String, 'demo', 10)
        timerPeriod = .5
        self.timer = self.create_timer(timerPeriod, self.timerCallback)
        self.i = 0
        self.message = ''
    
    def timerCallback(self):
        msg = String()

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
                if abs(event.value) > 0.1:
                    msg.data = str(event.value)
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: "{msg.data}"')


            # if event.type == JOYHATMOTION:

            #     msg.data = str(event)
            #     self.publisher_.publish(msg)
            #     self.get_logger().info(f'Publishing: "{msg.data}"')

            if event.type == QUIT:

                pygame.quit()

                sys.exit()

            if event.type == KEYDOWN:

                if event.key == K_ESCAPE:

                    pygame.quit()

                    sys.exit()

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
