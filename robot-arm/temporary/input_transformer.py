#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from input_mapping.msg import Actions
import json

class Controller:
    def __init__(self):
        # create a publisher for the arm actions
        # you could subscribe to this information using
        # rospy.Subscriber('arm_actions', Actions, callback)
        self.publisher = rospy.Publisher('arm_actions', Actions)

        # create node, 'anonymous' makes sure the IDs are distictive each time
        rospy.init_node('input_transformer', anonymous=True)

        # load the binding json
        with open('bindings.json') as binding_json:
            self.bindings = json.load(binding_json)

    def start(self):
        # function for parsing binding strings
        # ex input: 'buttons[11]', output: (buttons, 11)
        def parse_binding(binding):
            bracket_index = binding.index('[')
            action_type = binding[0:bracket_index]
            number = binding[bracket_index: len(binding) - 1]
            return action_type, number

        def joy_callback(data):
            # do stuff w/ data

            # determine the form of input being used (joystick vs controller)
            # and then publish the actions

            # create message
            actions_msg = Actions()
            # name of joint at index in actions_msg.joints
            joint_names = ["lift", "shoulder", "elbow"] #, "grip"]
            # array containing joint update values 
            actions_msg.joints = [0] * len(joint_names)


            # position is for inverse, currently not being used
            actions_msg.position = []
            actions_msg.forwards = True

            # default speed
            actions_msg.speed = 1
            
            input = ''
            # the joystick has 6 axes (and 11 buttons)
            if ((len(data.axes) == 6) and (len(data.buttons11 == 11))):
                input = 'joystick'
            else:
                # controlller code here, change else to elif,
                # and else to handle an unkown controller
                input = 'controller'
            
            # bind actions by iterating through the bindings dictionary
            for key in self.bindings:
                # if the key is a joint
                if key in joint_names:
                    binding = self.bindings[key][input]
                    action_type, number = parse_binding(binding)
                    actions_msg.joints[joint_names.index(key)] = data[action_type][number]
                # todo, maybe make this a toggle
                elif key == 'speed':
                    # determine the keys for 2x and .5x speeds
                    speed_double_input = self.bindings.speed['2'][input]
                    speed_half_input = self.bindings.speed['.5'][input]
                    double_type, double_number = parse_binding(speed_double_input)
                    half_type, half_number = parse_binding(speed_half_input)
                    # boolean values for whether half/double speeds are presssed
                    double = data[double_type][double_number]
                    half = data[half_type, half_number]
                    # if both pressed, stay at 1x so check that just one pressed
                    if double and not half:
                        actions_msg.speed = 2
                    elif half and not double:
                        actions_msg.speed = .5
                    else:
                        actions_msg.speed = 1

                # todo handle the grip parsing
            
            # publish the fully formed msg
            self.publisher.publish(actions_msg)
        
        # subscribe to joy, the source of raw controller input
        rospy.Subscriber("joy", Joy, joy_callback)

        # spin() keeps python from exiting until node is stopped
        rospy.spin()

if __name__ == '__main__':
    controller = Controller()
    controller.start()
        




