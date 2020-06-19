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

        # load the binding json
        with open('bindings.json') as binding_json:
            self.bindings = json.load(binding_json)

    def start(self):
        def joy_callback(data):
            # do stuff w/ data

            # determine the form of input being used (joystick vs controller
            # and then publish the actions

            actions_msg = Actions()
            actions_msg.joints = []
            actions_msg.position = []
            actions_msg.forwards = True
            
            # the joystick has 6 axes (and 11 buttons)
            if ((len(data.axes) == 6) and (len(data.buttons11 == 11)):
                joystick = True
            else:
                # controlller code here, change else to elif,
                # and else to handle an unkown controller
                joystick = False
            
            # bind actions by iterating through the bindings dictionary
            for key in bindings:
                pass # todo





            self.publisher.publish(actions_msg)
        
        rospy.Subscriber("joy", Joy, joy_callback)

        # spin() keeps python from exiting until node is stopped
        rospy.spin()

if __name__ == '__main__':
    controller = Controller()
    controller.start()
        




