#!/usr/bin/env python
from __future__ import print_function

import sys
import copy
from turtle import home
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import Joy
from roboclaw_node.srv import HomeArm
from controller_manager_msgs.srv import SwitchController


def move_to_coord(move_group, x, y, z):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.5
    pose_goal.orientation.y = -0.5
    pose_goal.orientation.z = -0.5
    pose_goal.orientation.w = 0.5

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

# front, default, or pickup
def move_to_named_position(group, pos_name):
    group.set_named_target(pos_name)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def home_arm(joints):
    rospy.wait_for_service('/brc_arm/home_arm')
    try:
        home_arm = rospy.ServiceProxy('/brc_arm/home_arm', HomeArm)
        ret = home_arm(joints)
        for i in range(0, len(joints)):
            if joints[i] == 1 and ret.status[i] == 0:
                rospy.logerr("Failed to home joint: %d", i)
                return 0
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)
    return 1


def switch_to_group_position_controller():
    rospy.wait_for_service('/brc_arm/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy(
            '/brc_arm/controller_manager/switch_controller', SwitchController)
        #ret = switch_controller(['/brc_arm/controller/position'], ['/brc_arm/controller/trajectory'], 2)
        ret = switch_controller(['/brc_arm/controller/position'],
                                ['/brc_arm/controller/trajectory'], 2, False, 0.0)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


def joy_callback(data):
    axes = data.axes
    buttons = data.buttons
    # print(axes)
    # Publish twist message to servo server
    twist = geometry_msgs.msg.TwistStamped()
    twist.header = data.header
    twist.header.frame_id = ''
    twist.twist.linear.x = data.axes[1]
    twist.twist.linear.y = data.axes[0]
    #twist.twist.linear.z = data.axes[7]
    if(data.axes[5] != 1.0):
        twist.twist.linear.z = -(data.axes[5] - 1) / 2
    else:
        twist.twist.linear.z = (data.axes[2] - 1) / 2
    twist.twist.angular.x = data.axes[3] * -5
    twist.twist.angular.y = data.axes[4] * 5
    twist.twist.angular.z = data.axes[6] * 5
    twist_pub.publish(twist)


def main():
    rospy.sleep(5)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("robot_arm")

    rospy.sleep(2)
    rospy.loginfo("Homing!")
    home_arm([0, 0, 1, 0, 0])

    move_to_named_position(arm_group, "front")
    #switch_to_group_position_controller()

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('brc_arm_node')
    global twist_pub
    twist_pub = rospy.Publisher(
        '/servo_server/delta_twist_cmds', geometry_msgs.msg.TwistStamped, queue_size=10)
    rospy.Subscriber("/joy", Joy, joy_callback)
    main()
    rospy.spin()
