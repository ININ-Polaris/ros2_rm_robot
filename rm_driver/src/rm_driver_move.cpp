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

void RmArm::Arm_MoveJ_Callback(rm_ros_interfaces::msg::Movej::SharedPtr msg) {
  float joint[7];
  int speed;
  int block;
  int32_t res;
  std_msgs::msg::UInt32 movej_data;
  std_msgs::msg::Bool movej_result;
  int trajectory_connect;

  for (int i = 0; i < 6; i++) {
    joint[i] = msg->joint[i] * RAD_DEGREE;
  }
  if (msg->dof == 7) {
    joint[6] = msg->joint[6] * RAD_DEGREE;
  }
  speed = msg->speed;
  trajectory_connect = msg->trajectory_connect;
  block = static_cast<int>(msg->block);
  // res = Rm_Api.Service_Movej_Cmd(m_sockhand, joint, v ,0, trajectory_connect, block);
  res = Rm_Api.rm_movej(robot_handle, joint, speed, 0, trajectory_connect, block);
  movej_data.data = res;
  if (movej_data.data == 0) {
    movej_result.data = true;
    this->MoveJ_Cmd_Result->publish(movej_result);
  } else {
    movej_result.data = false;
    this->MoveJ_Cmd_Result->publish(movej_result);
    RCLCPP_INFO(this->get_logger(), "MoveJ error code is %d\n", movej_data.data);
  }
}

void RmArm::Arm_MoveL_Callback(rm_ros_interfaces::msg::Movel::SharedPtr msg) {
  rm_pose_t pose;
  int speed;
  bool block;
  int32_t res;
  std_msgs::msg::UInt32 movel_data;
  std_msgs::msg::Bool movel_result;
  rm_quat_t rec_pose;
  rm_euler_t tarns_euler;
  int trajectory_connect;

  pose.position.x = msg->pose.position.x;
  pose.position.y = msg->pose.position.y;
  pose.position.z = msg->pose.position.z;
  rec_pose.w = msg->pose.orientation.w;
  rec_pose.x = msg->pose.orientation.x;
  rec_pose.y = msg->pose.orientation.y;
  rec_pose.z = msg->pose.orientation.z;
  // tarns_euler = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose);
  tarns_euler = Rm_Api.rm_algo_quaternion2euler(rec_pose);
  pose.euler.rx = tarns_euler.rx;
  pose.euler.ry = tarns_euler.ry;
  pose.euler.rz = tarns_euler.rz;
  speed = msg->speed;
  block = msg->block;
  trajectory_connect = msg->trajectory_connect;
  // res = Rm_Api.Service_Movel_Cmd(m_sockhand, pose, v ,0, trajectory_connect, block);
  res = Rm_Api.rm_movel(robot_handle, pose, speed, 0, trajectory_connect, static_cast<int>(block));
  movel_data.data = res;
  if (movel_data.data == 0) {
    movel_result.data = true;
    this->MoveL_Cmd_Result->publish(movel_result);
  } else {
    movel_result.data = false;
    this->MoveL_Cmd_Result->publish(movel_result);
    RCLCPP_INFO(this->get_logger(), "MoveL error code is %d\n", movel_data.data);
  }
}

void RmArm::Arm_MoveC_Callback(rm_ros_interfaces::msg::Movec::SharedPtr msg) {

  rm_pose_t pose_via;
  rm_pose_t pose_to;
  int speed;
  int loop;
  int32_t res;
  std_msgs::msg::UInt32 movec_data;
  std_msgs::msg::Bool movec_result;
  rm_quat_t rec_pose_via;
  rm_quat_t rec_pose_to;
  rm_euler_t tarns_euler_via;
  rm_euler_t tarns_euler_to;
  int trajectory_connect;
  bool block;

  pose_via.position.x = msg->pose_mid.position.x;
  pose_via.position.y = msg->pose_mid.position.y;
  pose_via.position.z = msg->pose_mid.position.z;
  rec_pose_via.w = msg->pose_mid.orientation.w;
  rec_pose_via.x = msg->pose_mid.orientation.x;
  rec_pose_via.y = msg->pose_mid.orientation.y;
  rec_pose_via.z = msg->pose_mid.orientation.z;
  // tarns_euler_via = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose_via);
  tarns_euler_via = Rm_Api.rm_algo_quaternion2euler(rec_pose_via);
  pose_via.euler.rx = tarns_euler_via.rx;
  pose_via.euler.ry = tarns_euler_via.ry;
  pose_via.euler.rz = tarns_euler_via.rz;

  pose_to.position.x = msg->pose_end.position.x;
  pose_to.position.y = msg->pose_end.position.y;
  pose_to.position.z = msg->pose_end.position.z;
  rec_pose_to.w = msg->pose_end.orientation.w;
  rec_pose_to.x = msg->pose_end.orientation.x;
  rec_pose_to.y = msg->pose_end.orientation.y;
  rec_pose_to.z = msg->pose_end.orientation.z;
  // tarns_euler_to = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose_to);
  tarns_euler_to = Rm_Api.rm_algo_quaternion2euler(rec_pose_to);
  pose_to.euler.rx = tarns_euler_to.rx;
  pose_to.euler.ry = tarns_euler_to.ry;
  pose_to.euler.rz = tarns_euler_to.rz;

  speed = msg->speed;
  loop = msg->loop;
  block = msg->block;
  trajectory_connect = msg->trajectory_connect;
  // res = Rm_Api.Service_Movec_Cmd(m_sockhand, pose_via, pose_to, v, 0, loop, trajectory_connect, block);
  res = Rm_Api.rm_movec(robot_handle, pose_via, pose_to, speed, 0, loop, trajectory_connect, static_cast<int>(block));
  movec_data.data = res;
  if (movec_data.data == 0) {
    movec_result.data = true;
    this->MoveC_Cmd_Result->publish(movec_result);
  } else {
    movec_result.data = false;
    this->MoveC_Cmd_Result->publish(movec_result);
    RCLCPP_INFO(this->get_logger(), "MoveC error code is %d\n", movec_data.data);
  }
}

void RmArm::Arm_Movej_CANFD_Callback(rm_ros_interfaces::msg::Jointpos::SharedPtr msg) {
  float joint[7];
  bool follow;
  float expand;
  int32_t res;
  int trajectory_mode;
  int radio;
  std_msgs::msg::UInt32 movej_CANFD_data;

  for (int i = 0; i < 6; i++) {
    joint[i] = msg->joint[i] * RAD_DEGREE;
  }
  if (msg->dof == 7) {
    joint[6] = msg->joint[6] * RAD_DEGREE;
  }

  follow = msg->follow;
  expand = msg->expand * RAD_DEGREE;
  trajectory_mode = trajectory_mode_;
  radio = radio_;
  // std::cout<<"Service_Movej_CANFD_With_Radio is run!!!!"<<std::endl;
  //  res = Rm_Api.Service_Movej_CANFD_With_Radio(m_sockhand, joint, follow, expand, trajectory_mode, radio);
  res = Rm_Api.rm_movej_canfd(robot_handle, joint, follow, expand, trajectory_mode, radio);

  movej_CANFD_data.data = res;
  if (movej_CANFD_data.data != 0) {
    RCLCPP_INFO(this->get_logger(), "Movej CANFD error code is %d\n", movej_CANFD_data.data);
  }
}

void RmArm::Arm_Movej_CANFD_Custom_Callback(rm_ros_interfaces::msg::Jointposcustom::SharedPtr msg) {
  float joint[7];
  bool follow;
  float expand;
  int32_t res;
  int trajectory_mode;
  int radio;
  std_msgs::msg::UInt32 movej_CANFD_data;

  for (int i = 0; i < 6; i++) {
    joint[i] = msg->joint[i] * RAD_DEGREE;
  }
  if (msg->dof == 7) {
    joint[6] = msg->joint[6] * RAD_DEGREE;
  }

  follow = msg->follow;
  expand = msg->expand * RAD_DEGREE;
  trajectory_mode = msg->trajectory_mode;
  radio = msg->radio;
  // std::cout<<"Service_Movej_CANFD_Trajectory is run!!!!"<<std::endl;
  // res = Rm_Api.Service_Movej_CANFD_With_Radio(m_sockhand, joint, follow, expand, trajectory_mode, radio);
  res = Rm_Api.rm_movej_canfd(robot_handle, joint, follow, expand, trajectory_mode, radio);

  movej_CANFD_data.data = res;
  if (movej_CANFD_data.data != 0) {
    RCLCPP_INFO(this->get_logger(), "Movej CANFD error code is %d\n", movej_CANFD_data.data);
  }
}

void RmArm::Arm_Movep_CANFD_Callback(rm_ros_interfaces::msg::Cartepos::SharedPtr msg) {

  rm_pose_t pose;
  bool follow;
  int32_t res;
  std_msgs::msg::UInt32 movep_CANFD_data;
  rm_quat_t rec_pose;
  rm_euler_t tarns_euler;
  int trajectory_mode;
  int radio;

  pose.position.x = msg->pose.position.x;
  pose.position.y = msg->pose.position.y;
  pose.position.z = msg->pose.position.z;
  rec_pose.w = msg->pose.orientation.w;
  rec_pose.x = msg->pose.orientation.x;
  rec_pose.y = msg->pose.orientation.y;
  rec_pose.z = msg->pose.orientation.z;
  // tarns_euler = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose);
  tarns_euler = Rm_Api.rm_algo_quaternion2euler(rec_pose);
  pose.euler.rx = tarns_euler.rx;
  pose.euler.ry = tarns_euler.ry;
  pose.euler.rz = tarns_euler.rz;
  follow = msg->follow;
  trajectory_mode = trajectory_mode_;
  radio = radio_;

  // res = Rm_Api.Service_Movep_CANFD_With_Radio(m_sockhand, pose, follow, trajectory_mode, radio);
  res = Rm_Api.rm_movep_canfd(robot_handle, pose, follow, trajectory_mode, radio);

  movep_CANFD_data.data = res;
  if (movep_CANFD_data.data != 0) {
    RCLCPP_INFO(this->get_logger(), "Movep CANFD error code is %d\n", movep_CANFD_data.data);
  }
}

void RmArm::Arm_Movep_CANFD_Custom_Callback(rm_ros_interfaces::msg::Carteposcustom::SharedPtr msg) {
  rm_pose_t pose;
  bool follow;
  int32_t res;
  std_msgs::msg::UInt32 movep_CANFD_data;
  rm_quat_t rec_pose;
  rm_euler_t tarns_euler;
  int trajectory_mode;
  int radio;

  pose.position.x = msg->pose.position.x;
  pose.position.y = msg->pose.position.y;
  pose.position.z = msg->pose.position.z;
  rec_pose.w = msg->pose.orientation.w;
  rec_pose.x = msg->pose.orientation.x;
  rec_pose.y = msg->pose.orientation.y;
  rec_pose.z = msg->pose.orientation.z;
  // tarns_euler = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose);
  tarns_euler = Rm_Api.rm_algo_quaternion2euler(rec_pose);
  pose.euler.rx = tarns_euler.rx;
  pose.euler.ry = tarns_euler.ry;
  pose.euler.rz = tarns_euler.rz;
  follow = msg->follow;
  trajectory_mode = msg->trajectory_mode;
  radio = msg->radio;
  std::cout << "Service_Movep_CANFD_Trajectory is run!!!!" << std::endl;
  // res = Rm_Api.Service_Movep_CANFD_With_Radio(m_sockhand, pose, follow, trajectory_mode, radio);
  res = Rm_Api.rm_movep_canfd(robot_handle, pose, follow, trajectory_mode, radio);
  movep_CANFD_data.data = res;
  if (movep_CANFD_data.data != 0) {
    RCLCPP_INFO(this->get_logger(), "Movep CANFD error code is %d\n", movep_CANFD_data.data);
  }
}

void RmArm::Arm_MoveJ_P_Callback(rm_ros_interfaces::msg::Movejp::SharedPtr msg) {

  rm_pose_t pose;
  int speed;
  bool block;
  int32_t res;
  std_msgs::msg::UInt32 movej_p_data;
  std_msgs::msg::Bool movej_p_result;
  rm_quat_t rec_pose;
  rm_euler_t tarns_euler;
  int trajectory_connect;

  pose.position.x = msg->pose.position.x;
  pose.position.y = msg->pose.position.y;
  pose.position.z = msg->pose.position.z;
  rec_pose.w = msg->pose.orientation.w;
  rec_pose.x = msg->pose.orientation.x;
  rec_pose.y = msg->pose.orientation.y;
  rec_pose.z = msg->pose.orientation.z;
  // tarns_euler = Rm_Api.Service_Algo_Quaternion2Euler(rec_pose);
  tarns_euler = Rm_Api.rm_algo_quaternion2euler(rec_pose);
  pose.euler.rx = tarns_euler.rx;
  pose.euler.ry = tarns_euler.ry;
  pose.euler.rz = tarns_euler.rz;
  speed = msg->speed;
  trajectory_connect = msg->trajectory_connect;
  block = msg->block;
  // res = Rm_Api.Service_Movej_P_Cmd(m_sockhand, pose, v ,0, trajectory_connect, block);
  res = Rm_Api.rm_movej_p(robot_handle, pose, speed, 0, trajectory_connect, block);
  movej_p_data.data = res;
  if (movej_p_data.data == 0) {
    movej_p_result.data = true;
    this->MoveJ_P_Cmd_Result->publish(movej_p_result);
  } else {
    movej_p_result.data = false;
    this->MoveJ_P_Cmd_Result->publish(movej_p_result);
    RCLCPP_INFO(this->get_logger(), "Movej_p error code is %d\n", movej_p_data.data);
  }
}

void RmArm::Arm_Move_Stop_Callback(const std_msgs::msg::Empty::SharedPtr msg) {
  copy = msg;
  // bool block;
  int32_t res;
  std_msgs::msg::UInt32 move_stop_data;
  std_msgs::msg::Bool move_stop_result;

  // block = msg->data;
  // res = Rm_Api.Service_Move_Stop_Cmd(m_sockhand, block);
  res = Rm_Api.rm_set_arm_stop(robot_handle);
  move_stop_data.data = res;
  if (move_stop_data.data == 0) {
    move_stop_result.data = true;
    this->Move_Stop_Cmd_Result->publish(move_stop_result);
  } else {
    move_stop_result.data = false;
    this->Move_Stop_Cmd_Result->publish(move_stop_result);
    RCLCPP_INFO(this->get_logger(), "Move stop error code is %d\n", move_stop_data.data);
  }
}
