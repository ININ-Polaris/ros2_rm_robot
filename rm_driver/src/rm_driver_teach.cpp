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

void RmArm::Set_Joint_Teach_Callback(rm_ros_interfaces::msg::Jointteach::SharedPtr msg)
{
    int num;
    int direction;
    int v;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 joint_teach_data;
    std_msgs::msg::Bool joint_teach_result;

    num = msg->num;
    direction = msg->direction;
    v = msg->speed;
    // block = msg->block;

    // res = Rm_Api.Service_Joint_Teach_Cmd(m_sockhand, num, direction, v , block);
    res = Rm_Api.rm_set_joint_teach(robot_handle, num, direction, v);
    joint_teach_data.data = res;
    if(joint_teach_data.data == 0)
    {
        joint_teach_result.data = true;
        this->Set_Joint_Teach_Cmd_Result->publish(joint_teach_result);
    }
    else
    {
        joint_teach_result.data = false;
        this->Set_Joint_Teach_Cmd_Result->publish(joint_teach_result);
        RCLCPP_INFO (this->get_logger(),"Joint_Teach error code is %d\n",joint_teach_data.data);
    }
}

void RmArm::Set_Pos_Teach_Callback(rm_ros_interfaces::msg::Posteach::SharedPtr msg)
{
    int type;
    int direction;
    int v;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 pos_teach_data;
    std_msgs::msg::Bool pos_teach_result;

    type = msg->type;
    direction = msg->direction;
    v = msg->speed;
    // block = msg->block;

    // res = Rm_Api.Service_Pos_Teach_Cmd(m_sockhand, (POS_TEACH_MODES)type, direction, v , block);
    res = Rm_Api.rm_set_pos_teach(robot_handle, (rm_pos_teach_type_e)type, direction, v);
    pos_teach_data.data = res;
    if(pos_teach_data.data == 0)
    {
        pos_teach_result.data = true;
        this->Set_Pos_Teach_Cmd_Result->publish(pos_teach_result);
    }
    else
    {
        pos_teach_result.data = false;
        this->Set_Pos_Teach_Cmd_Result->publish(pos_teach_result);
        RCLCPP_INFO (this->get_logger(),"Pos_Teach error code is %d\n",pos_teach_data.data);
    }
}

void RmArm::Set_Ort_Teach_Callback(rm_ros_interfaces::msg::Ortteach::SharedPtr msg)
{
    int type;
    int direction;
    int v;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 ort_teach_data;
    std_msgs::msg::Bool ort_teach_result;

    type = msg->type;
    direction = msg->direction;
    v = msg->speed;
    // block = msg->block;

    // res = Rm_Api.Service_Ort_Teach_Cmd(m_sockhand, (ORT_TEACH_MODES)type, direction, v , block);
    res = Rm_Api.rm_set_ort_teach(robot_handle, (rm_ort_teach_type_e)type, direction, v );
    ort_teach_data.data = res;
    if(ort_teach_data.data == 0)
    {
        ort_teach_result.data = true;
        this->Set_Ort_Teach_Cmd_Result->publish(ort_teach_result);
    }
    else
    {
        ort_teach_result.data = false;
        this->Set_Ort_Teach_Cmd_Result->publish(ort_teach_result);
        RCLCPP_INFO (this->get_logger(),"Ort_Teach error code is %d\n",ort_teach_data.data);
    }
}

void RmArm::Set_Stop_Teach_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    copy = msg;
    // bool block;
    int32_t res;
    std_msgs::msg::UInt32 stop_teach_data;
    std_msgs::msg::Bool stop_teach_result;

    // block = msg->data;

    // res = Rm_Api.Service_Teach_Stop_Cmd(m_sockhand, block);
    res = Rm_Api.rm_set_stop_teach(robot_handle);
    stop_teach_data.data = res;
    if(stop_teach_data.data == 0)
    {
        stop_teach_result.data = true;
        this->Set_Stop_Teach_Cmd_Result->publish(stop_teach_result);
    }
    else
    {
        stop_teach_result.data = false;
        this->Set_Stop_Teach_Cmd_Result->publish(stop_teach_result);
        RCLCPP_INFO (this->get_logger(),"Stop_Teach error code is %d\n",stop_teach_data.data);
    }
}
