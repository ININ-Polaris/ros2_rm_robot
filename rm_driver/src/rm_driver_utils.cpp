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


void RmArm::Arm_Get_Realtime_Push_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    int32_t res;
    rm_ros_interfaces::msg::Setrealtimepush Setrealtime_msg;
    copy = msg;
    rm_realtime_push_config_t config;
    // res = Rm_Api.Service_Get_Realtime_Push(m_sockhand, &config);
    res = Rm_Api.rm_get_realtime_push(robot_handle, &config);
    if(res == 0)
    {
        Setrealtime_msg.cycle = config.cycle;
        Setrealtime_msg.port = config.port;
        Setrealtime_msg.force_coordinate = config.force_coordinate;
        Setrealtime_msg.ip = config.ip;
        Setrealtime_msg.hand_enable = config.custom_config.hand_state;
        Setrealtime_msg.joint_speed_enable = config.custom_config.joint_speed;
        Setrealtime_msg.lift_state_enable = config.custom_config.lift_state;
        Setrealtime_msg.expand_state_enable = config.custom_config.expand_state;
        Setrealtime_msg.arm_current_status_enable = config.custom_config.arm_current_status;
        Setrealtime_msg.aloha_state_enable = config.custom_config.aloha_state;
        Setrealtime_msg.plus_base_enable = config.custom_config.plus_base;
        rm_plus_base_g = config.custom_config.plus_base;
        Setrealtime_msg.plus_state_enable = config.custom_config.plus_state;
        rm_plus_state_g = config.custom_config.plus_state;
        udp_hand_g = config.custom_config.hand_state;
        this->Get_Realtime_Push_Result->publish(Setrealtime_msg);
    }
    else
    RCLCPP_INFO (this->get_logger(),"The error code is %d\n",res);
}

void RmArm::Arm_Set_Realtime_Push_Callback(const rm_ros_interfaces::msg::Setrealtimepush::SharedPtr msg)
{
    rm_realtime_push_config_t config;
    int32_t res;
    std_msgs::msg::Bool set_realtime_result;
    config.port = msg->port ;
    config.cycle = msg->cycle;
    config.force_coordinate = msg->force_coordinate;
    config.enable = true;
    strcpy(config.ip,msg->ip.data());
    rm_udp_custom_config_t config_enable;
    config_enable.expand_state = msg->expand_state_enable;
    config_enable.hand_state = msg->hand_enable;
    udp_hand_g = msg->hand_enable;
    config_enable.joint_speed = msg->joint_speed_enable;
    config_enable.lift_state = msg->lift_state_enable;
    config_enable.arm_current_status = msg->arm_current_status_enable;
    config_enable.aloha_state = msg->aloha_state_enable;
    config_enable.plus_base = msg->plus_base_enable;
    rm_plus_base_g = msg->plus_base_enable;
    config_enable.plus_state = msg->plus_state_enable;
    rm_plus_state_g = msg->plus_state_enable;
    config.custom_config = config_enable;
    // res = Rm_Api.Service_Set_Realtime_Push(m_sockhand, config);
    res = Rm_Api.rm_set_realtime_push(robot_handle, config);
    if(res == 0)
    {
        set_realtime_result.data = true;
        this->Set_Realtime_Push_Result->publish(set_realtime_result);
    }
    else
    {
        set_realtime_result.data = false;
        this->Set_Realtime_Push_Result->publish(set_realtime_result);
        RCLCPP_INFO (this->get_logger(),"The error code is %d\n",res);
    }
}

void RmArm::Set_UDP_Configuration(int udp_cycle, int udp_port, int udp_force_coordinate, std::string udp_ip, bool hand, bool rm_plus_base, bool rm_plus_state)
{
    int32_t res;
    rm_realtime_push_config_t config;
    config.port = udp_port ;
    config.cycle = udp_cycle/5;
    config.force_coordinate = udp_force_coordinate;
    config.enable = true;
    strcpy(config.ip,udp_ip.data());
    rm_udp_custom_config_t config_enable;
    config_enable.expand_state = 0;
    config_enable.hand_state = hand;
    udp_hand_g = hand;
    config_enable.joint_speed = 0;
    config_enable.lift_state = 0;
    config_enable.aloha_state = 0;
    config_enable.plus_base = rm_plus_base;
    rm_plus_base_g = rm_plus_base;
    config_enable.plus_state = rm_plus_state;
    rm_plus_state_g = rm_plus_state;
    config_enable.arm_current_status = 0;
    config.custom_config = config_enable;
    // res = Rm_Api.Service_Set_Realtime_Push(m_sockhand, config);
    res = Rm_Api.rm_set_realtime_push(robot_handle, config);
    if(res == 0)
    {
        RCLCPP_INFO (this->get_logger(),"UDP_Configuration is cycle:%dms,port:%d,force_coordinate:%d,ip:%s,hand:%d,rm_plus_base:%d,rm_plus_state:%d\n", udp_cycle, udp_port, udp_force_coordinate, udp_ip.c_str(),udp_hand_g,rm_plus_base_g,rm_plus_state_g);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"The error code is %d\n",res);
    }
}

void RmArm::Get_Arm_Version()
{
    // ArmSoftwareInfo arm_software_info;
    rm_arm_software_version_t arm_software_info;
    char product_version[100];
    int32_t res;
    //res = Rm_Api.Service_Get_Arm_Software_Info(m_sockhand, &arm_software_info);
    res = Rm_Api.rm_get_arm_software_info(robot_handle, &arm_software_info);
    if(res == 0)
    {
        RCLCPP_INFO (this->get_logger(),"product_version = %s",arm_software_info.product_version);
        strcpy(product_version, arm_software_info.product_version);
        Udp_RM_Joint.control_version = 1;
        for(int i=0;i<10;i++)
        {
            if(product_version[i]=='F')
            {
                Udp_RM_Joint.control_version = 2;
            }
        }
        // RCLCPP_INFO (this->get_logger(),"control_version = %d",Udp_RM_Joint.control_version);
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Service_Get_Arm_Software_Version error = %d",res);
    }
}


void RmArm::Arm_Set_Tool_Voltage_Callback(const std_msgs::msg::UInt16::SharedPtr msg)
{
    int type;
    int32_t res;
    std_msgs::msg::Bool arm_set_tool_voltage_result;
    type = msg->data;
    // res = Rm_Api.Service_Set_Tool_Voltage(m_sockhand, type, RM_BLOCK);
    res = Rm_Api.rm_set_tool_voltage(robot_handle, type);
    if(res == 0)
    {
        arm_set_tool_voltage_result.data = true;
        this->Set_Tool_Voltage_Result->publish(arm_set_tool_voltage_result);
    }
    else
    {
        arm_set_tool_voltage_result.data = false;
        this->Set_Tool_Voltage_Result->publish(arm_set_tool_voltage_result);
        RCLCPP_INFO (this->get_logger(),"Arm set tool voltage error code is %d\n",res);
    }
}

void RmArm::Arm_Set_Joint_Err_Clear_Callback(const rm_ros_interfaces::msg::Jointerrclear::SharedPtr msg)
{
    int joint_num;
    // bool block;
    int32_t res;
    std_msgs::msg::Bool set_joint_err_clear_result;
    joint_num = msg->joint_num;
    // block = msg->block;
    // res = Rm_Api.Service_Set_Joint_Err_Clear(m_sockhand, joint_num, block);
    res = Rm_Api.rm_set_joint_clear_err(robot_handle, joint_num);

    if(res == 0)
    {
        set_joint_err_clear_result.data = true;
        this->Set_Joint_Err_Clear_Result->publish(set_joint_err_clear_result);
    }
    else
    {
        set_joint_err_clear_result.data = false;
        this->Set_Joint_Err_Clear_Result->publish(set_joint_err_clear_result);
        RCLCPP_INFO (this->get_logger(),"Arm set joint err clear callback error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Current_Arm_State_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    rm_pose_t pose;
    // float joint[7];
    // u_int16_t Err;
    // u_int8_t Err_len;
    rm_current_arm_state_t  current_state;
    int32_t res;
    copy = msg;
    std_msgs::msg::Bool get_current_arm_State_result;
    rm_euler_t euler;
    rm_quat_t quat;
    int i;
    // res = Rm_Api.Service_Get_Current_Arm_State(m_sockhand, joint, &pose, &Err, &Err_len);
    res = Rm_Api.rm_get_current_arm_state(robot_handle, &current_state);
    if(res == 0)
    {
        pose = current_state.pose;
        
        Arm_original_state.dof = 6;
        Arm_state.dof = 6;
        for(i=0;i<6;i++)
        {
            Arm_original_state.joint[i] = current_state.joint[i];
            Arm_state.joint[i] = current_state.joint[i] * DEGREE_RAD;
        }
        if(arm_dof_g == 7)
        {
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
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm get current arm state error code is %d\n",res);
    }
}

void RmArm::Arm_Clear_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    copy = msg;
    // bool block;
    int32_t res;
    std_msgs::msg::Bool clear_force_data_result;
    // block = msg->data;
    // res = Rm_Api.Service_Clear_Force_Data(m_sockhand, block);
    res = Rm_Api.rm_clear_force_data(robot_handle);
    
    if(res == 0)
    {
        clear_force_data_result.data = true;
        this->Clear_Force_Data_Result->publish(clear_force_data_result);
    }
    else
    {
        clear_force_data_result.data = false;
        this->Clear_Force_Data_Result->publish(clear_force_data_result);
        RCLCPP_INFO (this->get_logger(),"Arm clear force data error code is %d\n",res);
    }
}

void RmArm::Arm_Get_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg)
{
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
    if(res == 0)
    {
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
    }
    else
    {
        RCLCPP_INFO (this->get_logger(),"Arm get force data error code is %d\n",res);
    }
}

void Udp_Robot_Status_Callback(rm_realtime_arm_joint_state_t data)
{
    for(int i = 0; i < 6; i++)
    {
        Udp_RM_Joint.joint[i] = data.joint_status.joint_position[i];
        Udp_RM_Joint.err_flag[i] = data.joint_status.joint_err_code[i];
        Udp_RM_Joint.joint_current[i] = data.joint_status.joint_current[i];
        Udp_RM_Joint.en_flag[i] = data.joint_status.joint_en_flag[i];
        Udp_RM_Joint.joint_speed[i] = data.joint_status.joint_speed[i];
        Udp_RM_Joint.joint_temperature[i] = data.joint_status.joint_temperature[i];
        Udp_RM_Joint.joint_voltage[i] = data.joint_status.joint_voltage[i];
    }
    if(arm_dof_g == 7)
    {
        Udp_RM_Joint.joint[6] = data.joint_status.joint_position[6];
        Udp_RM_Joint.err_flag[6] = data.joint_status.joint_err_code[6];
        Udp_RM_Joint.joint_current[6] = data.joint_status.joint_current[6];
        Udp_RM_Joint.en_flag[6] = data.joint_status.joint_en_flag[6];
        Udp_RM_Joint.joint_speed[6] = data.joint_status.joint_speed[6];
        Udp_RM_Joint.joint_temperature[6] = data.joint_status.joint_temperature[6];
        Udp_RM_Joint.joint_voltage[6] = data.joint_status.joint_voltage[6];
    }
    if(Udp_RM_Joint.udp_rm_err.err_len != data.err.err_len)
    {
        Udp_RM_Joint.udp_rm_err.err_len = data.err.err_len;
        Udp_RM_Joint.udp_rm_err.err.resize(Udp_RM_Joint.udp_rm_err.err_len);
        for(int i = 0; i<Udp_RM_Joint.udp_rm_err.err_len; i++)
        {
            Udp_RM_Joint.udp_rm_err.err[i] = data.err.err[i];
        }
    }

    if(Udp_RM_Joint.control_version == 2)
    {
        for(int i = 0; i < 6; i++)
        {
            Udp_RM_Joint.six_force[i] = data.force_sensor.force[i];
            Udp_RM_Joint.zero_force[i] = data.force_sensor.zero_force[i];
        }
    }
    if(udp_hand_g == true)
    {
        for(int i = 0; i < 6; i++)
        {
            Udp_RM_Joint.hand_angle[i] = data.handState.hand_angle[i];
            Udp_RM_Joint.hand_force[i] = data.handState.hand_force[i];
            Udp_RM_Joint.hand_pos[i] = data.handState.hand_pos[i];
            Udp_RM_Joint.hand_state[i] = data.handState.hand_state[i];
        }
        Udp_RM_Joint.hand_err = data.handState.hand_err;
    }
    
    if(rm_plus_base_g == true)
    {
        for(int i = 0; i < 10; i++)
        {
            Udp_RM_Joint.udp_rm_plus_base_info.manu[i] = data.plus_base_info.manu[i];
            Udp_RM_Joint.udp_rm_plus_base_info.hv[i] = data.plus_base_info.hv[i];
            Udp_RM_Joint.udp_rm_plus_base_info.sv[i] = data.plus_base_info.sv[i];
            Udp_RM_Joint.udp_rm_plus_base_info.bv[i] = data.plus_base_info.bv[i];
            Udp_RM_Joint.udp_rm_plus_base_info.angle_low[i] = data.plus_base_info.angle_low[i];
            Udp_RM_Joint.udp_rm_plus_base_info.angle_up[i] = data.plus_base_info.angle_up[i];
            Udp_RM_Joint.udp_rm_plus_base_info.pos_up[i] = data.plus_base_info.pos_up[i];
            Udp_RM_Joint.udp_rm_plus_base_info.pos_low[i] = data.plus_base_info.pos_low[i];
            Udp_RM_Joint.udp_rm_plus_base_info.speed_up[i] = data.plus_base_info.speed_up[i];
            Udp_RM_Joint.udp_rm_plus_base_info.speed_low[i] = data.plus_base_info.speed_low[i];
            Udp_RM_Joint.udp_rm_plus_base_info.force_up[i] = data.plus_base_info.force_up[i];
            Udp_RM_Joint.udp_rm_plus_base_info.force_low[i] = data.plus_base_info.force_low[i];
        }
        for(int i = 0; i < 2; i++)
        {
            Udp_RM_Joint.udp_rm_plus_base_info.angle_low[10+i] = data.plus_base_info.angle_low[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.angle_up[10+i] = data.plus_base_info.angle_up[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.pos_up[10+i] = data.plus_base_info.pos_up[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.pos_low[10+i] = data.plus_base_info.pos_low[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.speed_up[10+i] = data.plus_base_info.speed_up[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.speed_low[10+i] = data.plus_base_info.speed_low[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.force_up[10+i] = data.plus_base_info.force_up[10+i];
            Udp_RM_Joint.udp_rm_plus_base_info.force_low[10+i] = data.plus_base_info.force_low[10+i];
        }
        Udp_RM_Joint.udp_rm_plus_base_info.id = data.plus_base_info.id;
        Udp_RM_Joint.udp_rm_plus_base_info.dof = data.plus_base_info.dof;
        Udp_RM_Joint.udp_rm_plus_base_info.check = data.plus_base_info.check;
        Udp_RM_Joint.udp_rm_plus_base_info.bee = data.plus_base_info.bee;
        Udp_RM_Joint.udp_rm_plus_base_info.force = data.plus_base_info.force;
        Udp_RM_Joint.udp_rm_plus_base_info.touch = data.plus_base_info.touch;
        Udp_RM_Joint.udp_rm_plus_base_info.touch_num = data.plus_base_info.touch_num;
        Udp_RM_Joint.udp_rm_plus_base_info.touch_sw = data.plus_base_info.touch_sw;
        Udp_RM_Joint.udp_rm_plus_base_info.hand = data.plus_base_info.hand;
    }

    if(rm_plus_state_g == true)
    {
        Udp_RM_Joint.udp_rm_plus_state_info.sys_state = data.plus_state_info.sys_state;
        for(int i = 0; i < 12; i++)
        {
            Udp_RM_Joint.udp_rm_plus_state_info.dof_state[i] = data.plus_state_info.dof_state[i];
            Udp_RM_Joint.udp_rm_plus_state_info.dof_err[i] = data.plus_state_info.dof_err[i];
            Udp_RM_Joint.udp_rm_plus_state_info.pos[i] = data.plus_state_info.pos[i];
            Udp_RM_Joint.udp_rm_plus_state_info.speed[i] = data.plus_state_info.speed[i];
            Udp_RM_Joint.udp_rm_plus_state_info.angle[i] = data.plus_state_info.angle[i];
            Udp_RM_Joint.udp_rm_plus_state_info.current[i] = data.plus_state_info.current[i];
            Udp_RM_Joint.udp_rm_plus_state_info.normal_force[i] = data.plus_state_info.normal_force[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tangential_force[i] = data.plus_state_info.tangential_force[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tangential_force_dir[i] = data.plus_state_info.tangential_force_dir[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tsa[i] = data.plus_state_info.tsa[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tma[i] = data.plus_state_info.tma[i];
            Udp_RM_Joint.udp_rm_plus_state_info.touch_data[i] = data.plus_state_info.touch_data[i];
            Udp_RM_Joint.udp_rm_plus_state_info.force[i] = data.plus_state_info.force[i];
        }
        for(int i = 12; i < 18; i++)
        {
            Udp_RM_Joint.udp_rm_plus_state_info.normal_force[i] = data.plus_state_info.normal_force[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tangential_force[i] = data.plus_state_info.tangential_force[i];
            Udp_RM_Joint.udp_rm_plus_state_info.tangential_force_dir[i] = data.plus_state_info.tangential_force_dir[i];
            Udp_RM_Joint.udp_rm_plus_state_info.touch_data[i] = data.plus_state_info.touch_data[i];
        }
    }

    Udp_RM_Joint.joint_position[0] = data.waypoint.position.x;
    Udp_RM_Joint.joint_position[1] = data.waypoint.position.y;
    Udp_RM_Joint.joint_position[2] = data.waypoint.position.z;
    
    Udp_RM_Joint.joint_quat[0] = data.waypoint.quaternion.w;
    Udp_RM_Joint.joint_quat[1] = data.waypoint.quaternion.x;
    Udp_RM_Joint.joint_quat[2] = data.waypoint.quaternion.y;
    Udp_RM_Joint.joint_quat[3] = data.waypoint.quaternion.z;

    Udp_RM_Joint.joint_euler[0] = data.waypoint.euler.rx;
    Udp_RM_Joint.joint_euler[1] = data.waypoint.euler.ry;
    Udp_RM_Joint.joint_euler[2] = data.waypoint.euler.rz;

    Udp_RM_Joint.arm_current_status = data.arm_current_status;
    //std::cout<<"callback arm_current_status is "<<Udp_RM_Joint.arm_current_status<<std::endl;

    if(Udp_RM_Joint.control_version == 3)
    {
        Udp_RM_Joint.one_force = data.force_sensor.force[0];
        Udp_RM_Joint.one_zero_force = data.force_sensor.zero_force[0];
    }
    // Udp_RM_Joint.sys_err = data.sys_err;
    // Udp_RM_Joint.arm_err = data.arm_err;
    Udp_RM_Joint.coordinate = data.force_sensor.coordinate;
    // if(udp_hand_g == true)
    // {
    
    // }
}
