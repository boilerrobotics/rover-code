# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

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


#initialize moveit_commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_pose_test',anonymous=True)

#instantiate robot commander object
#provides info on kinematic model and current 
#joint states
robot = moveit_commander.RobotCommander()

#instantiate PlanningSceneInterface
#provides remote interface for understanding
#of envrionment
scene = moveit_commander.PlanningSceneInterface()

#instantiate movegroupcommander, planning group
#for a group of joints, used to plan and execute motions
group_name = "robot_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

move_to_coord(move_group, 0.56, 0.0, 0.5)
move_to_coord(move_group, 0.56, 0.168, 0.8)
move_to_coord(move_group, 0.56, -0.54, 0.8)
move_to_coord(move_group, 0.29, 0.0, 0.4)
move_to_coord(move_group, 0.73, 0.0, 0.4)
move_to_coord(move_group, 0.56, 0.0, 0.5)




moveit_commander.roscpp_shutdown()

