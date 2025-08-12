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


void RmArm::Arm_Set_Hand_Posture_Callback(const rm_ros_interfaces::msg::Handposture::SharedPtr msg)
{
    int posture_num;
    bool block;
    int timeout;
    int32_t res;
    std_msgs::msg::Bool set_hand_posture_result;
    posture_num = msg->posture_num;
    block = msg->block;
    timeout = msg->timeout;
    // res = Rm_Api.Service_Set_Hand_Posture(m_sockhand, posture_num, block);
    res = Rm_Api.rm_set_hand_posture(robot_handle, posture_num, block, timeout);
    if(res == 0)
    {
        set_hand_posture_result.data = true;
        this->Set_Hand_Posture_Result->publish(set_hand_posture_result);
    }
    else
    {
        set_hand_posture_result.data = false;
        this->Set_Hand_Posture_Result->publish(set_hand_posture_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand posture error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Seq_Callback(const rm_ros_interfaces::msg::Handseq::SharedPtr msg)
{
    int seq_num;
    bool block;
    int timeout;
    int32_t res;
    std_msgs::msg::Bool set_hand_seq_result;
    seq_num = msg->seq_num;
    block = msg->block;
    timeout = msg->timeout;
    // res = Rm_Api.Service_Set_Hand_Seq(m_sockhand, seq_num, block);
    res = Rm_Api.rm_set_hand_seq(robot_handle, seq_num, block, timeout);
    if(res == 0)
    {
        set_hand_seq_result.data = true;
        this->Set_Hand_Seq_Result->publish(set_hand_seq_result);
    }
    else
    {
        set_hand_seq_result.data = false;
        this->Set_Hand_Seq_Result->publish(set_hand_seq_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand seq error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Angle_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg)
{
    int hand_data[12];
    uint32_t res = 0;
    rm_peripheral_read_write_params_t params;
    params.port = 1;
    params.address = 1486;
    params.device = 1;
    params.num = 6;
    // bool block;
    std_msgs::msg::Bool set_hand_angle_result;
    for(int i = 0;i<6;i++)
    {
        hand_data[i * 2] = msg->hand_angle[i] & 0xFF;
        hand_data[i * 2 + 1] = (msg->hand_angle[i] >> 8) & 0xFF;
        // hand_data[i * 2] = msg->hand_angle[i] & 0xFF;
        // hand_data[i * 2 + 1] = (msg->hand_angle[i] >> 8) & 0xFF;
    }
    // block = msg->block;
    //res = Rm_Api.Service_Set_Hand_Angle(m_sockhand, angle, block);
    RCLCPP_INFO (this->get_logger(),"Arm set hand angle %d, %d, %d, %d, %d, %d\n", msg->hand_angle[0], msg->hand_angle[1], msg->hand_angle[2], msg->hand_angle[3], msg->hand_angle[4], msg->hand_angle[5]);
    RCLCPP_INFO (this->get_logger(),"Arm set hand data %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", hand_data[0], hand_data[1], hand_data[2], hand_data[3], hand_data[4], hand_data[5], hand_data[6], hand_data[7], hand_data[8], hand_data[9], hand_data[10], hand_data[11]);
    // RCLCPP_INFO (this->get_logger(),"int: %ld, short: %ld, long: %ld, float: %ld, double: %ld, char: %ld\n", sizeof(int), sizeof(short), sizeof(long), sizeof(float), sizeof(double), sizeof(char));
    res = Rm_Api.rm_write_registers(robot_handle, params, hand_data);
    if(res == 0)
    {
        set_hand_angle_result.data = true;
        this->Set_Hand_Angle_Result->publish(set_hand_angle_result);
    }
    else
    {
        set_hand_angle_result.data = false;
        this->Set_Hand_Angle_Result->publish(set_hand_angle_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand angle error code is %d\n", res);
    }

    // for (int i = 0; i < 6; i++) {
    //     uint32_t res = 0;
    //     res = Rm_Api.rm_write_single_register(robot_handle, {.port = 1, .address = 1486 + 2 * i, .device = 1, .num = 1}, hand_angle[i]);
    //     if(res == 0)
    //     {
    //         set_hand_angle_result.data = true;
    //         this->Set_Hand_Angle_Result->publish(set_hand_angle_result);
    //     }
    //     else
    //     {
    //         set_hand_angle_result.data = false;
    //         this->Set_Hand_Angle_Result->publish(set_hand_angle_result);
    //         RCLCPP_INFO (this->get_logger(),"Arm set hand angle error code is %d, id is %d\n", res, i);
    //     }
    // }
}

void RmArm::Arm_Set_Hand_Speed_Callback(const rm_ros_interfaces::msg::Handspeed::SharedPtr msg)
{
    int speed;
    //bool block;
    int32_t res;
    std_msgs::msg::Bool set_hand_speed_result;
    speed = msg->hand_speed;
    // block = msg->block;
    // res = Rm_Api.Service_Set_Hand_Speed(m_sockhand, speed, block);
    res = Rm_Api.rm_set_hand_speed(robot_handle, speed);
    if(res == 0)
    {
        set_hand_speed_result.data = true;
        this->Set_Hand_Speed_Result->publish(set_hand_speed_result);
    }
    else
    {
        set_hand_speed_result.data = false;
        this->Set_Hand_Speed_Result->publish(set_hand_speed_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand speed error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Force_Callback(const rm_ros_interfaces::msg::Handforce::SharedPtr msg)
{
    int force;
    // bool block;
    int32_t res;
    std_msgs::msg::Bool set_hand_force_result;
    force = msg->hand_force;
    // block = msg->block;
    // res = Rm_Api.Service_Set_Hand_Force(m_sockhand, force, block);
    res = Rm_Api.rm_set_hand_force(robot_handle, force);
    if(res == 0)
    {
        set_hand_force_result.data = true;
        this->Set_Hand_Force_Result->publish(set_hand_force_result);
    }
    else
    {
        set_hand_force_result.data = false;
        this->Set_Hand_Force_Result->publish(set_hand_force_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand force error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Follow_Angle_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg)
{
    int angle[6];
    int block;
    int32_t res;
    std_msgs::msg::Bool set_hand_angle_result;
    for(int i = 0;i<6;i++)
    {
        angle[i] = msg->hand_angle[i];
    }
    block = msg->block;
    // res = Rm_Api.Service_Set_Hand_Follow_Angle(m_sockhand, angle, block);
    res = Rm_Api.rm_set_hand_follow_angle(robot_handle, angle, block);
    if(res == 0)
    {
        set_hand_angle_result.data = true;
        this->Set_Hand_Follow_Angle_Result->publish(set_hand_angle_result);
    }
    else
    {
        set_hand_angle_result.data = false;
        this->Set_Hand_Follow_Angle_Result->publish(set_hand_angle_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand follow angle error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Hand_Follow_Pos_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg)
{
    int pos[6];
    int block;
    int32_t res;
    std_msgs::msg::Bool set_hand_pos_result;
    for(int i = 0;i<6;i++)
    {
        pos[i] = msg->hand_angle[i];
    }
    block = msg->block;
    // res = Rm_Api.Service_Set_Hand_Follow_Pos(m_sockhand, pos, block);
    res = Rm_Api.rm_set_hand_follow_pos(robot_handle, pos, block);
    if(res == 0)
    {
        set_hand_pos_result.data = true;
        this->Set_Hand_Follow_Pos_Result->publish(set_hand_pos_result);
    }
    else
    {
        set_hand_pos_result.data = false;
        this->Set_Hand_Follow_Pos_Result->publish(set_hand_pos_result);
        RCLCPP_INFO (this->get_logger(),"Arm set hand follow angle error code is %d\n",res);
    }
}
