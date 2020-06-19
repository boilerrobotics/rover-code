#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from X.msg import Actions


class Controller:
    __init__(self):
        # create a publisher for the arm actions
        # you could subscribe to this information using
        # rospy.Subscriber('arm_actions', Actions, callback)
        self.publisher = rospy.Publisher('arm_actions', Actions)

        # create node
        rospy.init_node('input_transformer', anonymous=True)

    def start(self):
        def joy_callback(data):
            # do stuff w/ data
            # and then publish the actions
            actions_msg = Actions()
            actions_msg.joints = []
            actions_msg.position = []
            actions_msg.forwards = True
            self.publisher.publish(actions_msg)
        
        rospy.Subscriber("joy", Joy, joy_callback)

        # spin() keeps python from exiting until node is stopped
        rospy.spin()

if __name__ == '__main__':
    controller = Controller()
    controller.start()
        




