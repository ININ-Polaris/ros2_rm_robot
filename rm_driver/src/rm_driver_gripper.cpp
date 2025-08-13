// Copyright (c) 2024  RealMan Intelligent Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_driver.h"
#include <cstdint>

using namespace std::chrono_literals;

void RmArm::Arm_Set_Gripper_Pick_On_Callback(const rm_ros_interfaces::msg::Gripperpick::SharedPtr msg) {
  int speed;
  int force;
  bool block;
  int timeout;
  int32_t res;
  std_msgs::msg::Bool set_gripper_pick_on_result;
  speed = msg->speed;
  force = msg->force;
  block = msg->block;
  timeout = msg->timeout;
  // res = Rm_Api.Service_Set_Gripper_Pick_On(m_sockhand, speed, force, block, timeout);
  res = Rm_Api.rm_set_gripper_pick_on(robot_handle, speed, force, block, timeout);
  if (res == 0) {
    set_gripper_pick_on_result.data = true;
    this->Set_Gripper_Pick_On_Result->publish(set_gripper_pick_on_result);
  } else {
    set_gripper_pick_on_result.data = false;
    this->Set_Gripper_Pick_On_Result->publish(set_gripper_pick_on_result);
    RCLCPP_INFO(this->get_logger(), "Arm set gripper pick on error code is %d\n", res);
  }
}

void RmArm::Arm_Set_Gripper_Pick_Callback(const rm_ros_interfaces::msg::Gripperpick::SharedPtr msg) {
  int speed;
  int force;
  bool block;
  int timeout;
  int32_t res;
  std_msgs::msg::Bool set_gripper_pick_result;
  speed = msg->speed;
  force = msg->force;
  block = msg->block;
  timeout = msg->timeout;
  // res = Rm_Api.Service_Set_Gripper_Pick(m_sockhand, speed, force, block, timeout);
  res = Rm_Api.rm_set_gripper_pick(robot_handle, speed, force, block, timeout);
  if (res == 0) {
    set_gripper_pick_result.data = true;
    this->Set_Gripper_Pick_Result->publish(set_gripper_pick_result);
  } else {
    set_gripper_pick_result.data = false;
    this->Set_Gripper_Pick_Result->publish(set_gripper_pick_result);
    RCLCPP_INFO(this->get_logger(), "Arm set gripper pick error code is %d\n", res);
  }
}

void RmArm::Arm_Set_Gripper_Position_Callback(const rm_ros_interfaces::msg::Gripperset::SharedPtr msg) {
  int position;
  bool block;
  int timeout;
  int32_t res;
  std_msgs::msg::Bool set_gripper_position_result;
  position = msg->position;
  block = msg->block;
  timeout = msg->timeout;
  // res = Rm_Api.Service_Set_Gripper_Position(m_sockhand, position, block, timeout);
  res = Rm_Api.rm_set_gripper_position(robot_handle, position, block, timeout);
  if (res == 0) {
    set_gripper_position_result.data = true;
    this->Set_Gripper_Position_Result->publish(set_gripper_position_result);
  } else {
    set_gripper_position_result.data = false;
    this->Set_Gripper_Position_Result->publish(set_gripper_position_result);
    RCLCPP_INFO(this->get_logger(), "Arm set gripper position error code is %d\n", res);
  }
}
