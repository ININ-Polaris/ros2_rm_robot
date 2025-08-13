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

void RmArm::Arm_Change_Work_Frame_Callback(const std_msgs::msg::String::SharedPtr msg) {
  // FRAME_NAME work_frame;
  const char *work_name = msg->data.c_str();
  int32_t res;
  std_msgs::msg::Bool arm_change_work_frame_result;
  // strcpy(*work_name, msg->data.c_str());
  // res = Rm_Api.Service_Change_Work_Frame(m_sockhand, work_frame.name, RM_BLOCK);
  res = Rm_Api.rm_change_work_frame(robot_handle, work_name);
  if (res == 0) {
    arm_change_work_frame_result.data = true;
    this->Change_Work_Frame_Result->publish(arm_change_work_frame_result);
  } else {
    arm_change_work_frame_result.data = false;
    this->Change_Work_Frame_Result->publish(arm_change_work_frame_result);
    RCLCPP_INFO(this->get_logger(), "Arm_change_work_frame_callback error code is %d\n", res);
  }
}

void RmArm::Arm_Get_Curr_WorkFrame_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  // FRAME frame;
  rm_frame_t work_frame;
  int32_t res;
  std_msgs::msg::String curr_frame;
  std_msgs::msg::Bool arm_change_work_frame_result;
  copy = msg;
  // memset(frame.frame_name.name,'\0',sizeof(frame.frame_name.name));
  // res = Rm_Api.Service_Get_Current_Work_Frame(m_sockhand, &frame);
  res = Rm_Api.rm_get_current_work_frame(robot_handle, &work_frame);
  if (res == 0) {
    curr_frame.data = work_frame.frame_name;
    this->Get_Curr_WorkFrame_Result->publish(curr_frame);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm_get_curr_workFrame_callback error code is %d\n", res);
  }
}

void RmArm::Arm_Get_Current_Tool_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  // FRAME frame;
  rm_frame_t tool_frame;
  int32_t res;
  std_msgs::msg::String curr_frame;
  std_msgs::msg::Bool arm_change_work_frame_result;
  copy = msg;
  // memset(frame.frame_name.name,'\0',sizeof(frame.frame_name.name));
  //  res = Rm_Api.Service_Get_Current_Tool_Frame(m_sockhand, &frame);
  res = Rm_Api.rm_get_current_tool_frame(robot_handle, &tool_frame);
  if (res == 0) {
    curr_frame.data = tool_frame.frame_name;
    this->Get_Current_Tool_Frame_Result->publish(curr_frame);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm_get_curr_workFrame_callback error code is %d\n", res);
  }
}

void RmArm::Arm_Get_All_Tool_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  // FRAME_NAME name[10];
  rm_frame_name_t frame_names[10];
  int32_t res;
  rm_ros_interfaces::msg::Getallframe all_tool_frame;
  int len = -1;
  copy = msg;
  res = Rm_Api.rm_get_total_tool_frame(robot_handle, frame_names, &len);
  if (res == 0 && len <= 10) {
    for (int i = 0; i < len; i++) {
      // RCLCPP_INFO (this->get_logger(),"Arm all tool frame is %s\n",frame_names[i].name);
      all_tool_frame.frame_name[i] = std::string(frame_names[i].name);
    }
    for (int i = len; i < 10; i++) {
      all_tool_frame.frame_name[i] = "";
    }
    this->Get_All_Tool_Frame_Result->publish(all_tool_frame);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm_get_all_tool_frame_callback error code is %d\n", res);
  }
}

void RmArm::Arm_Get_All_Work_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  // char name[10];
  rm_frame_name_t frame_names[10];
  int32_t res;
  rm_ros_interfaces::msg::Getallframe all_work_frame;
  int len = -1;
  // for(int i = 0;i<=9;i++)
  // {
  //     memset(name[i].name,'\0',sizeof(name[i].name));
  // }
  copy = msg;
  // res = Rm_Api.Service_Get_All_Work_Frame(m_sockhand, name, &len);
  res = Rm_Api.rm_get_total_work_frame(robot_handle, frame_names, &len);
  if (res == 0 && len <= 10) {
    for (int i = 0; i <= len; i++) {
      // RCLCPP_INFO (this->get_logger(),"Arm all work frame is %s\n",frame_names[i].name);
      all_work_frame.frame_name[i] = std::string(frame_names[i].name);
    }
    for (int i = len; i < 10; i++) {
      // RCLCPP_INFO (this->get_logger(),"Arm all work frame is %s\n",frame_names[i].name);
      all_work_frame.frame_name[i] = "";
    }
    this->Get_All_Work_Frame_Result->publish(all_work_frame);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm_get_all_work_frame_callback error code is %d\n", res);
  }
}
