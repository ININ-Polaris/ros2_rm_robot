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

void RmArm::Arm_Get_Realtime_Push_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  int32_t res;
  rm_ros_interfaces::msg::Setrealtimepush Setrealtime_msg;
  copy = msg;
  rm_realtime_push_config_t config;
  // res = Rm_Api.Service_Get_Realtime_Push(m_sockhand, &config);
  res = Rm_Api.rm_get_realtime_push(robot_handle, &config);
  if (res == 0) {
    Setrealtime_msg.cycle = config.cycle;
    Setrealtime_msg.port = config.port;
    Setrealtime_msg.force_coordinate = config.force_coordinate;
    Setrealtime_msg.ip = config.ip;
    Setrealtime_msg.hand_enable = (config.custom_config.hand_state != 0);
    Setrealtime_msg.joint_speed_enable = (config.custom_config.joint_speed != 0);
    Setrealtime_msg.lift_state_enable = (config.custom_config.lift_state != 0);
    Setrealtime_msg.expand_state_enable = (config.custom_config.expand_state != 0);
    Setrealtime_msg.arm_current_status_enable = (config.custom_config.arm_current_status != 0);
    Setrealtime_msg.aloha_state_enable = (config.custom_config.aloha_state != 0);
    Setrealtime_msg.plus_base_enable = (config.custom_config.plus_base != 0);
    rm_plus_base_g = (config.custom_config.plus_base != 0);
    Setrealtime_msg.plus_state_enable = (config.custom_config.plus_state != 0);
    rm_plus_state_g = (config.custom_config.plus_state != 0);
    udp_hand_g = (config.custom_config.hand_state != 0);
    this->Get_Realtime_Push_Result->publish(Setrealtime_msg);
  } else {
    RCLCPP_INFO(this->get_logger(), "The error code is %d\n", res);
  }
}

void RmArm::Arm_Set_Realtime_Push_Callback(const rm_ros_interfaces::msg::Setrealtimepush::SharedPtr msg) {
  rm_realtime_push_config_t config;
  int32_t res;
  std_msgs::msg::Bool set_realtime_result;
  config.port = msg->port;
  config.cycle = msg->cycle;
  config.force_coordinate = msg->force_coordinate;
  config.enable = true;
  strcpy(config.ip, msg->ip.data());
  rm_udp_custom_config_t config_enable;
  config_enable.expand_state = static_cast<int>(msg->expand_state_enable);
  config_enable.hand_state = static_cast<int>(msg->hand_enable);
  udp_hand_g = msg->hand_enable;
  config_enable.joint_speed = static_cast<int>(msg->joint_speed_enable);
  config_enable.lift_state = static_cast<int>(msg->lift_state_enable);
  config_enable.arm_current_status = static_cast<int>(msg->arm_current_status_enable);
  config_enable.aloha_state = static_cast<int>(msg->aloha_state_enable);
  config_enable.plus_base = static_cast<int>(msg->plus_base_enable);
  rm_plus_base_g = msg->plus_base_enable;
  config_enable.plus_state = static_cast<int>(msg->plus_state_enable);
  rm_plus_state_g = msg->plus_state_enable;
  config.custom_config = config_enable;
  // res = Rm_Api.Service_Set_Realtime_Push(m_sockhand, config);
  res = Rm_Api.rm_set_realtime_push(robot_handle, config);
  if (res == 0) {
    set_realtime_result.data = true;
    this->Set_Realtime_Push_Result->publish(set_realtime_result);
  } else {
    set_realtime_result.data = false;
    this->Set_Realtime_Push_Result->publish(set_realtime_result);
    RCLCPP_INFO(this->get_logger(), "The error code is %d\n", res);
  }
}

void RmArm::Set_UDP_Configuration(int udp_cycle, int udp_port, int udp_force_coordinate, std::string udp_ip, bool hand,
                                  bool rm_plus_base, bool rm_plus_state) {
  int32_t res;
  rm_realtime_push_config_t config;
  config.port = udp_port;
  config.cycle = udp_cycle / 5;
  config.force_coordinate = udp_force_coordinate;
  config.enable = true;
  strcpy(config.ip, udp_ip.data());
  rm_udp_custom_config_t config_enable;
  config_enable.expand_state = 0;
  config_enable.hand_state = static_cast<int>(hand);
  udp_hand_g = hand;
  config_enable.joint_speed = 0;
  config_enable.lift_state = 0;
  config_enable.aloha_state = 0;
  config_enable.plus_base = static_cast<int>(rm_plus_base);
  rm_plus_base_g = rm_plus_base;
  config_enable.plus_state = static_cast<int>(rm_plus_state);
  rm_plus_state_g = rm_plus_state;
  config_enable.arm_current_status = 0;
  config.custom_config = config_enable;
  // res = Rm_Api.Service_Set_Realtime_Push(m_sockhand, config);
  res = Rm_Api.rm_set_realtime_push(robot_handle, config);
  if (res == 0) {
    RCLCPP_INFO(
        this->get_logger(),
        "UDP_Configuration is cycle:%dms,port:%d,force_coordinate:%d,ip:%s,hand:%d,rm_plus_base:%d,rm_plus_state:%d\n",
        udp_cycle, udp_port, udp_force_coordinate, udp_ip.c_str(), udp_hand_g, rm_plus_base_g, rm_plus_state_g);
  } else {
    RCLCPP_INFO(this->get_logger(), "The error code is %d\n", res);
  }
}

void RmArm::Get_Arm_Version() {
  // ArmSoftwareInfo arm_software_info;
  rm_arm_software_version_t arm_software_info;
  char product_version[100];
  int32_t res;
  // res = Rm_Api.Service_Get_Arm_Software_Info(m_sockhand, &arm_software_info);
  res = Rm_Api.rm_get_arm_software_info(robot_handle, &arm_software_info);
  if (res == 0) {
    RCLCPP_INFO(this->get_logger(), "product_version = %s", arm_software_info.product_version);
    strcpy(product_version, arm_software_info.product_version);
    Udp_RM_Joint.control_version = 1;
    for (int i = 0; i < 10; i++) {
      if (product_version[i] == 'F') {
        Udp_RM_Joint.control_version = 2;
      }
    }
    // RCLCPP_INFO (this->get_logger(),"control_version = %d",Udp_RM_Joint.control_version);
  } else {
    RCLCPP_INFO(this->get_logger(), "Service_Get_Arm_Software_Version error = %d", res);
  }
}

void RmArm::Arm_Set_Tool_Voltage_Callback(const std_msgs::msg::UInt16::SharedPtr msg) {
  int type;
  int32_t res;
  std_msgs::msg::Bool arm_set_tool_voltage_result;
  type = msg->data;
  // res = Rm_Api.Service_Set_Tool_Voltage(m_sockhand, type, RM_BLOCK);
  res = Rm_Api.rm_set_tool_voltage(robot_handle, type);
  if (res == 0) {
    arm_set_tool_voltage_result.data = true;
    this->Set_Tool_Voltage_Result->publish(arm_set_tool_voltage_result);
  } else {
    arm_set_tool_voltage_result.data = false;
    this->Set_Tool_Voltage_Result->publish(arm_set_tool_voltage_result);
    RCLCPP_INFO(this->get_logger(), "Arm set tool voltage error code is %d\n", res);
  }
}

void RmArm::Arm_Set_Joint_Err_Clear_Callback(const rm_ros_interfaces::msg::Jointerrclear::SharedPtr msg) {
  int joint_num;
  // bool block;
  int32_t res;
  std_msgs::msg::Bool set_joint_err_clear_result;
  joint_num = msg->joint_num;
  // block = msg->block;
  // res = Rm_Api.Service_Set_Joint_Err_Clear(m_sockhand, joint_num, block);
  res = Rm_Api.rm_set_joint_clear_err(robot_handle, joint_num);

  if (res == 0) {
    set_joint_err_clear_result.data = true;
    this->Set_Joint_Err_Clear_Result->publish(set_joint_err_clear_result);
  } else {
    set_joint_err_clear_result.data = false;
    this->Set_Joint_Err_Clear_Result->publish(set_joint_err_clear_result);
    RCLCPP_INFO(this->get_logger(), "Arm set joint err clear callback error code is %d\n", res);
  }
}

void RmArm::Arm_Get_Current_Arm_State_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  rm_pose_t pose;
  // float joint[7];
  // u_int16_t Err;
  // u_int8_t Err_len;
  rm_current_arm_state_t current_state;
  int32_t res;
  copy = msg;
  std_msgs::msg::Bool get_current_arm_State_result;
  rm_euler_t euler;
  rm_quat_t quat;
  // res = Rm_Api.Service_Get_Current_Arm_State(m_sockhand, joint, &pose, &Err, &Err_len);
  res = Rm_Api.rm_get_current_arm_state(robot_handle, &current_state);
  if (res == 0) {
    pose = current_state.pose;

    Arm_original_state.dof = 6;
    Arm_state.dof = 6;
    for (int i = 0; i < 6; i++) {
      Arm_original_state.joint[i] = current_state.joint[i];
      Arm_state.joint[i] = current_state.joint[i] * DEGREE_RAD;
    }
    if (arm_dof_g == 7) {
      Arm_original_state.joint[6] = current_state.joint[6];
      Arm_state.joint[6] = current_state.joint[6] * DEGREE_RAD;
      Arm_original_state.dof = 7;
      Arm_state.dof = 7;
    }

    Arm_original_state.pose[0] = pose.position.x;
    Arm_original_state.pose[1] = pose.position.y;
    Arm_original_state.pose[2] = pose.position.z;
    Arm_original_state.pose[3] = pose.euler.rx;
    Arm_original_state.pose[4] = pose.euler.ry;
    Arm_original_state.pose[5] = pose.euler.rz;
    Arm_original_state.err = *current_state.err.err;
    Arm_original_state.err_len = current_state.err.err_len;
    this->Get_Current_Arm_Original_State_Result->publish(Arm_original_state);

    euler.rx = pose.euler.rx;
    euler.ry = pose.euler.ry;
    euler.rz = pose.euler.rz;
    // quat = Rm_Api.Service_Algo_Euler2Quaternion(euler);
    quat = Rm_Api.rm_algo_euler2quaternion(euler);
    Arm_state.pose.orientation.w = quat.w;
    Arm_state.pose.orientation.x = quat.x;
    Arm_state.pose.orientation.y = quat.y;
    Arm_state.pose.orientation.z = quat.z;
    Arm_state.pose.position.x = pose.position.x;
    Arm_state.pose.position.y = pose.position.y;
    Arm_state.pose.position.z = pose.position.z;
    Arm_state.err = *current_state.err.err;
    Arm_state.err_len = current_state.err.err_len;
    this->Get_Current_Arm_State_Result->publish(Arm_state);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm get current arm state error code is %d\n", res);
  }
}

void RmArm::Arm_Clear_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  copy = msg;
  // bool block;
  int32_t res;
  std_msgs::msg::Bool clear_force_data_result;
  // block = msg->data;
  // res = Rm_Api.Service_Clear_Force_Data(m_sockhand, block);
  res = Rm_Api.rm_clear_force_data(robot_handle);

  if (res == 0) {
    clear_force_data_result.data = true;
    this->Clear_Force_Data_Result->publish(clear_force_data_result);
  } else {
    clear_force_data_result.data = false;
    this->Clear_Force_Data_Result->publish(clear_force_data_result);
    RCLCPP_INFO(this->get_logger(), "Arm clear force data error code is %d\n", res);
  }
}

void RmArm::Arm_Get_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  rm_ros_interfaces::msg::Sixforce force;
  rm_ros_interfaces::msg::Sixforce zero_force;
  rm_ros_interfaces::msg::Sixforce work_zero;
  rm_ros_interfaces::msg::Sixforce tool_zero;
  // float force_data[6];
  // float zero_force_data[6];
  // float work_zero_data[6];
  // float tool_zero_data[6];
  rm_force_data_t data;
  copy = msg;
  int32_t res;
  // res = Rm_Api.Service_Get_Force_Data(m_sockhand, force_data, zero_force_data, work_zero_data, tool_zero_data);
  res = Rm_Api.rm_get_force_data(robot_handle, &data);
  if (res == 0) {
    force.force_fx = data.force_data[0];
    force.force_fy = data.force_data[1];
    force.force_fz = data.force_data[2];
    force.force_mx = data.force_data[3];
    force.force_my = data.force_data[4];
    force.force_mz = data.force_data[5];
    Get_Force_Data_Result->publish(force);
    zero_force.force_fx = data.zero_force_data[0];
    zero_force.force_fy = data.zero_force_data[1];
    zero_force.force_fz = data.zero_force_data[2];
    zero_force.force_mx = data.zero_force_data[3];
    zero_force.force_my = data.zero_force_data[4];
    zero_force.force_mz = data.zero_force_data[5];
    Get_Zero_Force_Result->publish(zero_force);
    work_zero.force_fx = data.work_zero_force_data[0];
    work_zero.force_fy = data.work_zero_force_data[1];
    work_zero.force_fz = data.work_zero_force_data[2];
    work_zero.force_mx = data.work_zero_force_data[3];
    work_zero.force_my = data.work_zero_force_data[4];
    work_zero.force_mz = data.work_zero_force_data[5];
    Get_Work_Zero_Result->publish(work_zero);
    tool_zero.force_fx = data.tool_zero_force_data[0];
    tool_zero.force_fy = data.tool_zero_force_data[1];
    tool_zero.force_fz = data.tool_zero_force_data[2];
    tool_zero.force_mx = data.tool_zero_force_data[3];
    tool_zero.force_my = data.tool_zero_force_data[4];
    tool_zero.force_mz = data.tool_zero_force_data[5];
    Get_Tool_Zero_Result->publish(tool_zero);
  } else {
    RCLCPP_INFO(this->get_logger(), "Arm get force data error code is %d\n", res);
  }
}
