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

void RmArm::Arm_Start_Force_Position_Move_Callback(const std_msgs::msg::Empty::SharedPtr msg) {

  int32_t res;
  std_msgs::msg::Bool arm_start_force_result;
  copy = msg;
  // res = Rm_Api.Service_Start_Force_Position_Move(m_sockhand, true);
  res = Rm_Api.rm_start_force_position_move(robot_handle);
  if (res == 0) {
    arm_start_force_result.data = true;
    this->Start_Force_Position_Move_Result->publish(arm_start_force_result);
  } else {
    arm_start_force_result.data = false;
    this->Start_Force_Position_Move_Result->publish(arm_start_force_result);
  }
}

void RmArm::Arm_Stop_Force_Position_Move_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  int32_t res;
  std_msgs::msg::Bool arm_stop_force_result;
  copy = msg;
  // res = Rm_Api.Service_Stop_Force_Position_Move(m_sockhand, true);
  res = Rm_Api.rm_stop_force_position_move(robot_handle);
  if (res == 0) {
    arm_stop_force_result.data = true;
    this->Stop_Force_Position_Move_Result->publish(arm_stop_force_result);
  } else {
    arm_stop_force_result.data = false;
    this->Stop_Force_Position_Move_Result->publish(arm_stop_force_result);
  }
}

void RmArm::Arm_Force_Position_Move_Joint_Callback(
    const rm_ros_interfaces::msg::Forcepositionmovejoint::SharedPtr msg) {
  int32_t res;
  float joint[7];
  int sensor;
  int mode;
  std_msgs::msg::Bool force_position_move_joint_result;
  int dir;
  float force;
  bool follow;
  for (int i = 0; i < 6; i++) {
    joint[i] = msg->joint[i] * RAD_DEGREE;
  }
  if (msg->dof == 7) {
    joint[6] = msg->joint[6] * RAD_DEGREE;
  }

  sensor = msg->sensor;
  mode = msg->mode;
  dir = msg->dir;
  force = msg->force;
  follow = msg->follow;
  // res = Rm_Api.Service_Force_Position_Move_Joint(m_sockhand, joint, sensor, mode, dir, force, follow);
  res = Rm_Api.rm_force_position_move_joint(robot_handle, joint, sensor, mode, dir, force, follow);
  if (res != 0) {
    RCLCPP_INFO(this->get_logger(), "Arm force position move joint error code is %d\n", res);
  }
}

void RmArm::Arm_Force_Position_Move_Pose_Callback(const rm_ros_interfaces::msg::Forcepositionmovepose::SharedPtr msg) {
  int32_t res;
  rm_pose_t joint_pose;
  int sensor;
  int mode;
  rm_quat_t qua;
  rm_euler_t euler;
  int dir;
  float force;
  bool follow;
  qua.w = msg->pose.orientation.w;
  qua.x = msg->pose.orientation.x;
  qua.y = msg->pose.orientation.y;
  qua.z = msg->pose.orientation.z;
  // euler = Rm_Api.Service_Algo_Quaternion2Euler(qua);
  euler = Rm_Api.rm_algo_quaternion2euler(qua);
  joint_pose.position.x = msg->pose.position.x;
  joint_pose.position.y = msg->pose.position.y;
  joint_pose.position.z = msg->pose.position.z;
  joint_pose.euler.rx = euler.rx;
  joint_pose.euler.ry = euler.ry;
  joint_pose.euler.rz = euler.rz;
  sensor = msg->sensor;
  mode = msg->mode;
  dir = msg->dir;
  force = msg->force;
  follow = msg->follow;
  // res = Rm_Api.Service_Force_Position_Move_Pose(m_sockhand, joint_pose, sensor, mode, dir, force, follow);
  res = Rm_Api.rm_force_position_move_pose(robot_handle, joint_pose, sensor, mode, dir, force, follow);
  if (res != 0) {
    RCLCPP_INFO(this->get_logger(), "Arm force position move pose error code is %d\n", res);
  }
}

void RmArm::Arm_Set_Force_Postion_Callback(const rm_ros_interfaces::msg::Setforceposition::SharedPtr msg) {
  int32_t res;
  std_msgs::msg::Bool arm_set_force_postion_result;
  int sensor;
  int mode;
  int direction;
  int Force;
  // bool block;
  sensor = msg->sensor;
  mode = msg->mode;
  direction = msg->direction;
  Force = msg->n;
  // block = msg->block;
  // res = Rm_Api.Service_Set_Force_Postion(m_sockhand, sensor, mode, direction, N, block);
  res = Rm_Api.rm_set_force_position(robot_handle, sensor, mode, direction, Force);
  if (res == 0) {
    arm_set_force_postion_result.data = true;
    this->Set_Force_Postion_Result->publish(arm_set_force_postion_result);
  } else {
    arm_set_force_postion_result.data = false;
    this->Set_Force_Postion_Result->publish(arm_set_force_postion_result);
    RCLCPP_INFO(this->get_logger(), "Arm_set_force_postion_callback error code is %d\n", res);
  }
}
void RmArm::Arm_Stop_Force_Postion_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  copy = msg;
  int32_t res;
  std_msgs::msg::Bool arm_stop_force_postion_result;
  // bool block;
  // block = msg->data;
  // res = Rm_Api.Service_Stop_Force_Postion(m_sockhand, block);
  res = Rm_Api.rm_stop_force_position(robot_handle);
  if (res == 0) {
    arm_stop_force_postion_result.data = true;
    this->Stop_Force_Postion_Result->publish(arm_stop_force_postion_result);
  } else {
    arm_stop_force_postion_result.data = false;
    this->Stop_Force_Postion_Result->publish(arm_stop_force_postion_result);
    RCLCPP_INFO(this->get_logger(), "Arm stop force postion error code is %d\n", res);
  }
}
