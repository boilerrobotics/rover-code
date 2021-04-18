#!/usr/bin/env python
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
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

def switch_to_group_position_controller():
    rospy.wait_for_service('/brc_arm/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('/brc_arm/controller_manager/switch_controller', SwitchController)
        #ret = switch_controller(['/brc_arm/controller/position'], ['/brc_arm/controller/trajectory'], 2)
        ret = switch_controller(['/brc_arm/controller/position'], ['/brc_arm/controller/trajectory'], 0, False, 0.0)
    except rospy.ServiceException:
        print("Service call failed")


def main():
    rospy.sleep(10)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("robot_arm")
    # INSERT HOMING HERE!!
    move_to_named_position(arm_group, "front")
    switch_to_group_position_controller()
     
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('brc_arm_node')
    main()