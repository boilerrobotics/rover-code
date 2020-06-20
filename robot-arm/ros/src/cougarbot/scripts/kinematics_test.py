#!/usr/bin/env python

import numpy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import MoveGroupActionFeedback
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list
from enum import Enum
from copy import copy, deepcopy
import tf

class Mode(Enum):
    forwards = 1
    inverse = 2


class ArmController:
    def __init__(self, mode = Mode.forwards):

        self.state = "IDLE"
        self.delay = 0
        self.data = {}
        self.increment = .1
        self.inverse_scale = 5
        self.mode = mode
        group_name = "arm"
        #test
        # init moveit and ros node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('forward_kinematics_test', anonymous=True)

        # init robot model to get joint states / kinematic model
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        # get the move group associated w/ the arm we are using
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # set move group tolerances 
        move_group.set_goal_position_tolerance(.005)
        move_group.set_goal_orientation_tolerance((.01))


        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def start(self):
        def callback_joy(data):
            self.data = data

        rospy.Subscriber("joy", Joy, callback_joy)

        def callback_feedback(data):
            self.state = data.feedback.state         
        
        rospy.Subscriber("move_group/feedback", MoveGroupActionFeedback, callback_feedback)
        
        while not rospy.core.is_shutdown():
            if self.state == "IDLE" and self.data:

                if self.mode == Mode.forwards:
                    hip = self.data.axes[6]
                    shoulder = self.data.axes[1]
                    elbow = self.data.axes[4]
                    wrist = self.data.axes[7]

                    new_values =  self.move_group.get_current_joint_values()
                    original_values = new_values[:]

                    if hip > 0:
                        new_values[0] += self.increment
                    elif hip < 0:
                        new_values[0] -= self.increment

                    if shoulder > 0:
                        new_values[1] += self.increment
                    elif shoulder < 0:
                        new_values[1] -= self.increment

                    if elbow > 0:
                        new_values[2] += self.increment
                    elif elbow < 0:
                        new_values[2] -= self.increment

                    if wrist > 0:
                        new_values[3] += self.increment
                    elif wrist < 0:
                        new_values[3] -= self.increment

                    
                    if original_values != new_values:
                        self.state = "RUNNING"
                        self.go_to_joint_state(*new_values)
                elif self.mode == Mode.inverse:

                    new_pose = self.move_group.get_current_pose().pose
                    original_pose = deepcopy(new_pose)

                    # using approach from moveit source code
                    # https://github.com/ros-planning/moveit/blob/master/moveit_ros/visualization/src/moveit_ros_visualization/moveitjoy_module.py
                    left_analog_x = self.data.axes[0]
                    left_analog_y = self.data.axes[1]
                    right_analog_x = self.data.axes[3]
                    right_analog_y = self.data.axes[4]
                    left_bumper = self.data.buttons[4]
                    right_bumper = self.data.buttons[5]

                    dist = left_analog_y * left_analog_y + left_analog_x * left_analog_x

                    def signedSquare(val):
                        if val > 0:
                            sign = 1
                        else:
                            sign = -1
                        return val * val * sign

                    
                    x_diff = signedSquare(left_analog_y) / self.inverse_scale 
                    y_diff = signedSquare(left_analog_x) / self.inverse_scale 

                    if left_bumper:
                        z_diff = .005
                    elif right_bumper:
                        z_diff = -.005
                    else:
                        z_diff = 0

                    # may need to scale to 4, depending on history
                    z_scale = 2 

                    # array containing local x,y,z position change
                    local_move = numpy.array((x_diff, y_diff, z_diff * z_scale, 1))

                    q = numpy.array((original_pose.orientation.x, 
                                    original_pose.orientation.y,
                                    original_pose.orientation.z, 
                                    original_pose.orientation.w))

                    xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q),
                         local_move)

                    new_pose.position.x = original_pose.position.x + xyz_move[0]
                    new_pose.position.y = original_pose.position.y + xyz_move[1]
                    new_pose.position.z = original_pose.position.z + xyz_move[2]

                    roll = 0
                    pitch = 0
                    yaw = 0

                    # add controls for these here

                    diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                    new_q = tf.transformations.quaternion_multiply(q, diff_q)
                    new_pose.orientation.x = new_q[0]
                    new_pose.orientation.y = new_q[1]
                    new_pose.orientation.z = new_q[2]
                    new_pose.orientation.w = new_q[3]

                    
                    #x = self.data.axes[6] * self.inverse_scale
                    #y = self.data.axes[7] * self.inverse_scale
                    #z_up = self.data.buttons[5] * self.inverse_scale
                    #z_down = self.data.buttons[4] * self.inverse_scale

                    #print(x,y,z_up,z_down)
                    
                    # pose_goal.position.x = pose_goal.position.x + x
                    # pose_goal.position.y = pose_goal.position.y + y
                    # pose_goal.position.z = pose_goal.position.z + z_up - z_down

                    same_position = (new_pose.position.x == original_pose.position.x and
                    new_pose.position.y == original_pose.position.y and 
                    new_pose.position.z == original_pose.position.z)

                    if not same_position:
                        print "here"
                        print("original_goal", original_pose)
                        print("pose_goal", new_pose)
                        self.state = "RUNNING"
                        self.move_group.set_pose_target(new_pose)
                        self.move_group.go(wait=True)
                        self.move_group.stop()
                        self.move_group.clear_pose_targets()

                        




    # test function for arm controller
    def go_to_joint_state(self, hip = 0, shoulder = -.5 , elbow = .5 , wrist = 0):

        joint_goal = self.move_group.get_current_joint_values()
        '''
        rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["hip", "shoulder", "elbow", "wrist"], 
        points: [{positions: [0, -0.5, 0.5, 0], time_from_start: [.5, 0.0]}]}'
        '''
        # joints ["hip", "shoulder", "elbow", "wrist"]
        joint_goal[0] = hip
        joint_goal[1] = shoulder
        joint_goal[2] = elbow
        joint_goal[3] = wrist
    
        self.move_group.go(joint_goal, wait=False)

        # Calling ``stop()`` ensures that there is no residual movement
        #self.move_group.stop()



if __name__ == "__main__":
    arm_controller =  ArmController(mode = Mode.forwards)
    arm_controller.start()
    #arm_controller.go_to_joint_state()
    
