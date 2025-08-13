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

void RmArm::Arm_Set_Lift_Speed_Callback(const rm_ros_interfaces::msg::Liftspeed::SharedPtr msg) {
  int speed;
  int32_t res;
  std_msgs::msg::Bool set_lift_speed_result;
  speed = msg->speed;
  // res = Rm_Api.Service_Set_Lift_Speed(m_sockhand, speed);
  res = Rm_Api.rm_set_lift_speed(robot_handle, speed);
  if (res == 0) {
    set_lift_speed_result.data = true;
    this->Set_Lift_Speed_Result->publish(set_lift_speed_result);
  } else {
    set_lift_speed_result.data = false;
    this->Set_Lift_Speed_Result->publish(set_lift_speed_result);
    RCLCPP_INFO(this->get_logger(), "Arm set lift speed result error code is %d\n", res);
  }
}

void RmArm::Arm_Set_Lift_Height_Callback(const rm_ros_interfaces::msg::Liftheight::SharedPtr msg) {
  int speed;
  int height;
  bool block;
  int32_t res;
  std_msgs::msg::Bool set_lift_height_result;
  speed = msg->speed;
  height = msg->height;
  block = msg->block;
  // res = Rm_Api.Service_Set_Lift_Height(m_sockhand, height, speed, block);
  res = Rm_Api.rm_set_lift_height(robot_handle, speed, height, static_cast<int>(block));
  if (res == 0) {
    set_lift_height_result.data = true;
    this->Set_Lift_Height_Result->publish(set_lift_height_result);
  } else {
    set_lift_height_result.data = false;
    this->Set_Lift_Height_Result->publish(set_lift_height_result);
    RCLCPP_INFO(this->get_logger(), "Arm set lift height result error code is %d\n", res);
  }
}

void RmArm::Arm_Get_Lift_State_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  // int current;
  // int height;
  // int err_flag;
  // int mode;
  int32_t res;
  rm_ros_interfaces::msg::Liftstate lift_state;
  rm_expand_state_t state;
  copy = msg;
  // res = Rm_Api.Service_Get_Lift_State(m_sockhand, &height, &current, &err_flag, &mode);
  res = Rm_Api.rm_get_lift_state(robot_handle, &state);
  if (res == 0) {
    lift_state.current = state.current;
    lift_state.height = state.pos;
    lift_state.err_flag = state.err_flag;
    lift_state.mode = state.mode;
    // lift_state.current = current;
    // lift_state.height = height;
    // lift_state.err_flag = err_flag;
    // lift_state.mode = mode;
    this->Get_Lift_State_Result->publish(lift_state);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm set lift state result error code is %d\n", res);
  }
}
