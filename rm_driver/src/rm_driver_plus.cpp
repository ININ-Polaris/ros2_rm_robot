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

#include <cstdint>
#include <rm_driver/rm_driver.h>

using namespace std::chrono_literals;

void RmArm::Arm_Set_Rm_Plus_Mode_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
  int mode;
  int32_t res;
  std_msgs::msg::Bool set_rm_plus_mode;
  mode = msg->data;
  // res = Rm_Api.Service_Set_Lift_Height(m_sockhand, height, speed, block);
  res = Rm_Api.rm_set_rm_plus_mode(robot_handle, mode);
  if (res == 0) {
    set_rm_plus_mode.data = true;
    this->Set_Rm_Plus_Mode_Result->publish(set_rm_plus_mode);
  } else {
    set_rm_plus_mode.data = false;
    this->Set_Rm_Plus_Mode_Result->publish(set_rm_plus_mode);
    RCLCPP_INFO(this->get_logger(), "Arm set rm plus mode result error code is %d\n", res);
  }
}

void RmArm::Arm_Get_Rm_Plus_Mode_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  int32_t mode;
  int32_t res;
  std_msgs::msg::Int32 get_rm_plus_mode;
  copy = msg;
  // res = Rm_Api.Service_Get_Lift_State(m_sockhand, &height, &current, &err_flag, &mode);
  res = Rm_Api.rm_get_rm_plus_mode(robot_handle, &mode);
  if (res == 0) {
    get_rm_plus_mode.data = mode;
    this->Get_Rm_Plus_Mode_Result->publish(get_rm_plus_mode);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm get rm plus mode result error code is %d\n", res);
  }
}

void RmArm::Arm_Set_Rm_Plus_Touch_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
  int mode;
  int32_t res;
  std_msgs::msg::Bool set_rm_plus_touch;
  mode = msg->data;
  // res = Rm_Api.Service_Set_Lift_Height(m_sockhand, height, speed, block);
  res = Rm_Api.rm_set_rm_plus_touch(robot_handle, mode);
  if (res == 0) {
    set_rm_plus_touch.data = true;
    this->Set_Rm_Plus_Touch_Result->publish(set_rm_plus_touch);
  } else {
    set_rm_plus_touch.data = false;
    this->Set_Rm_Plus_Touch_Result->publish(set_rm_plus_touch);
    RCLCPP_INFO(this->get_logger(), "Arm set rm plus touch result error code is %d\n", res);
  }
}

void RmArm::Arm_Get_Rm_Plus_Touch_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  int32_t mode;
  int32_t res;
  std_msgs::msg::Int32 get_rm_plus_touch;
  copy = msg;
  // res = Rm_Api.Service_Get_Lift_State(m_sockhand, &height, &current, &err_flag, &mode);
  res = Rm_Api.rm_get_rm_plus_touch(robot_handle, &mode);
  if (res == 0) {
    get_rm_plus_touch.data = mode;
    this->Get_Rm_Plus_Touch_Result->publish(get_rm_plus_touch);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm get rm plus touch result error code is %d\n", res);
  }
}
