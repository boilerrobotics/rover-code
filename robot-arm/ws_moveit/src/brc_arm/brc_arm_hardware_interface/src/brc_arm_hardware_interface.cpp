/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <brc_arm_hardware_interface/brc_arm_hardware_interface.h>
#include <sstream>

#define NUM_ARM_JOINTS 7

namespace brc_arm_hardware_interface
{

BRCArmHardwareInterface::BRCArmHardwareInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  // runs the initialization stuff in the boilerplate library as well

  // Setup message to publish
  //ros::Publisher motor_pub = nh.advertise<roboclaw_node::MotorPosition>("motor_commands", 20);
  //motor_sub = nh.subscribe("\\brc_arm\\motor_positions", 10, BRCArmHardwareInterface::positionCallback);
  motor_sub = nh.subscribe("motor_positions", 10, &BRCArmHardwareInterface::positionCallback, this);
  ROS_INFO_NAMED("brc_arm_hardware_interface", "BRCArmHardwareInterface Ready.");
}

void BRCArmHardwareInterface::read(ros::Duration& elapsed_time)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void BRCArmHardwareInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  //for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
  // joint_position_[joint_id] = joint_position_command_[joint_id];
   //ROS_INFO_NAMED("controller", "%f", joint_position_command_[0]);
  //}

  roboclaw_node::MotorPosition msg;
  std::vector<int32_t> motor_num;
  std::vector<float> motor_angle;

//   # this message contains information for setting encoder positions
// int32[] motor_number
// int32[] accel
// int32[] speed
// int32[] deccel
// float32[] angle
  
  for (std::size_t joint_id = 0; joint_id < NUM_ARM_JOINTS; ++joint_id) {
    motor_num.push_back(joint_id);
    motor_angle.push_back(joint_position_command_[joint_id]);
  }

  msg.motor_number = motor_num;
  msg.angle = motor_angle;

  if(motor_pub.getNumSubscribers() > 0)
  {
    motor_pub.publish(msg);
  }


  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void BRCArmHardwareInterface::enforceLimits(ros::Duration& period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void BRCArmHardwareInterface::positionCallback(const roboclaw_node::EncoderValues::ConstPtr& msg) {
  //ROS_INFO("received enc pos");
  for (std::size_t joint_id = 0; joint_id < NUM_ARM_JOINTS; ++joint_id) {
   joint_position_[joint_id] = msg->angles.at(joint_id);
   //ROS_INFO_NAMED("controller", "%f", joint_position_command_[0]);
  }
}

}  // namespace brc_arm_hardware_interface
