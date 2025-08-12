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

//机械臂型号信息
int realman_arm;
//tcp ip
char* tcp_ip;
//tcp port
int tcp_port;
//udp hz
int udp_cycle_g = 5;
//arm dof
int arm_dof_g = 6;
//ctrl+c触发信号
bool ctrl_flag = false;
// 灵巧手数据发布
bool udp_hand_g = false;
// 末端设备基础信息发布
bool rm_plus_base_g = false;
// 末端设备实时信息
bool rm_plus_state_g = false;
//api类
RM_Service Rm_Api;
//机械臂TCp网络通信套接字
// SOCKHANDLE m_sockhand = -1;
//机械臂控制句柄
rm_robot_handle *robot_handle;

std_msgs::msg::UInt16 sys_err_;                                     //系统错误信息
std_msgs::msg::UInt16 arm_err_;                                     //机械臂错误信息
std_msgs::msg::UInt16 arm_coordinate_;                              //六维力基准坐标系
sensor_msgs::msg::JointState udp_real_joint_;                       //关节角度
geometry_msgs::msg::Pose udp_arm_pose_;                             //位姿
rm_ros_interfaces::msg::Sixforce udp_sixforce_;                     //六维力传感器原始数据
rm_ros_interfaces::msg::Sixforce udp_zeroforce_;                    //六维力传感器转化后数据
rm_ros_interfaces::msg::Sixforce udp_oneforce_;                     //一维力传感器原始数据
rm_ros_interfaces::msg::Sixforce udp_onezeroforce_;                 //一维力传感器转化后数据
rm_ros_interfaces::msg::Jointerrorcode udp_joint_error_code_;       //关节报错数据
rm_ros_interfaces::msg::Handstatus udp_hand_status_;
rm_ros_interfaces::msg::Armoriginalstate Arm_original_state;        //机械臂原始数据（角度+欧拉角）
rm_ros_interfaces::msg::Armstate Arm_state;                         //机械臂数据（弧度+四元数）
rm_ros_interfaces::msg::Armcurrentstatus udp_arm_current_status_;   //
rm_ros_interfaces::msg::Jointcurrent udp_joint_current_;            //
rm_ros_interfaces::msg::Jointenflag udp_joint_en_flag_;
rm_ros_interfaces::msg::Jointposeeuler udp_joint_pose_euler_;
rm_ros_interfaces::msg::Jointspeed udp_joint_speed_;
rm_ros_interfaces::msg::Jointtemperature udp_joint_temperature_; 
rm_ros_interfaces::msg::Jointvoltage udp_joint_voltage_;
rm_ros_interfaces::msg::Rmplusbase udp_rm_plus_base_;                   //末端设备基础信息
rm_ros_interfaces::msg::Rmplusstate udp_rm_plus_state_;                 //末端设备实时信息
rm_ros_interfaces::msg::Rmerr udp_rm_err_;                              //udp报错信息

JOINT_STATE_VALUE Udp_RM_Joint;

using namespace std::chrono_literals;

static void my_handler(int sig)  // can be called asynchronously
{ 
    (void)sig;
    ctrl_flag = true; // set flag
}

//连接机械臂网络   
int Arm_Socket_Start_Connect(void)
{
    int Arm_Socket;                         //机械臂TCp网络通信套接字
    int Arm_connect;                        //机械臂TCP连接状态

    Arm_Socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (Arm_Socket <= 0)
    {
        return 2;
    }

    struct sockaddr_in serAddr;
    // struct timeval tm;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(tcp_port);
    serAddr.sin_addr.s_addr = inet_addr(tcp_ip);
    int flag = 0;
    int old_flag = 0;
    flag |= O_NONBLOCK;
    // 设置为非阻塞模式
    old_flag = flag = fcntl(Arm_Socket, F_SETFL, O_NONBLOCK );
    // 查看连接状态
    Arm_connect = connect(Arm_Socket, (struct sockaddr *)&serAddr, sizeof(serAddr));
    // ROS_INFO("Arm_connect=%d\n",Arm_connect);
    if (Arm_connect != 0)
    {
        if(errno != EINPROGRESS) //connect返回错误。
		{
			std::cout<<"Arm_connect="<< Arm_connect <<"connect failed"<<std::endl;
            close(Arm_Socket);
            return 3;
		}
        else
        {
            struct timeval tm;  

			tm.tv_sec = 2;      
			tm.tv_usec = 0;

			fd_set wset;

			FD_ZERO(&wset);

			FD_SET(Arm_Socket,&wset); 
			int res = select(Arm_Socket+1, NULL, &wset, NULL, &tm);
            if(res <= 0)
			{
				std::cout<<"********************Connect faile check your connect!**************"<<std::endl;
				close(Arm_Socket);
				return 3;
			}

            if(FD_ISSET(Arm_Socket,&wset))
			{

				int err = -1;
				socklen_t len = sizeof(int);

				if(getsockopt(Arm_Socket, SOL_SOCKET, SO_ERROR, &err, &len ) < 0) //两种错误处理方式
				{
					std::cout<<"errno :" << errno << strerror(errno) <<std::endl;
					close(Arm_Socket);
					return 4;
				}
 
				if(err)
				{
					std::cout<<"********************Connect faile check your connect!**************"<<std::endl;
					errno = err;
					close(Arm_Socket);
					return 5;
				}
			}

        }

    }
    fcntl(Arm_Socket, F_SETFL, old_flag); //最后恢复sock的阻塞属性。
    close(Arm_Socket);
    return 0;
}

int Arm_Start(void)
{
    std::string version;
    Rm_Api.rm_init(RM_TRIPLE_MODE_E);
    //m_sockhand =  Rm_Api.Service_Arm_Socket_Start((char*)tcp_ip, tcp_port, 5000);
    version = Rm_Api.rm_api_version();
    // std::cout << version.c_str() << std::endl;
    // Rm_Api.rm_set_log_call_back(NULL ,0);
    robot_handle = Rm_Api.rm_create_robot_arm((char*)tcp_ip, tcp_port);

    
    // Rm_Api.rm_set_robot_dof(robot_handle,7);
    // robot_handle = Rm_Api.rm_create_robot_arm("192.168.1.18", 8080);
    if(robot_handle->id == -1)
    {
        //rm_delete_robot_arm(robot_handle);
        std::cout<<"arm connect err..."<<std::endl;
    }
    else if(robot_handle != NULL)
    {
        std::cout<<"connect success, arm id :"<<robot_handle->id<<std::endl;
    }
    // m_sockhand =  Rm_Api.Service_Arm_Socket_Start((char*)"192.168.1.234", 8080, 5000);
    //std::cout<<m_sockhand<<std::endl;
    return 0;
}

void Arm_Close(void)
{
    //Rm_Api.Service_Arm_Socket_Close(m_sockhand);
    Rm_Api.rm_delete_robot_arm(robot_handle);
}


void UdpPublisherNode::udp_timer_callback() 
{
    if(connect_state == 0)
    {
        // Rm_Api.Service_Realtime_Arm_Joint_State(Udp_RobotStatuscallback);
        Rm_Api.rm_realtime_arm_state_call_back(Udp_Robot_Status_Callback);
        udp_joint_error_code_.dof = 6;
        for(int i = 0;i<6;i++)
        {
            udp_real_joint_.position[i] = Udp_RM_Joint.joint[i] * DEGREE_RAD;
            udp_joint_error_code_.joint_error[i] = Udp_RM_Joint.err_flag[i];
            udp_joint_current_.joint_current[i] = Udp_RM_Joint.joint_current[i];
            udp_joint_en_flag_.joint_en_flag[i] = Udp_RM_Joint.en_flag[i];
            udp_joint_speed_.joint_speed[i] = Udp_RM_Joint.joint_speed[i];
            udp_joint_temperature_.joint_temperature[i] = Udp_RM_Joint.joint_temperature[i];
            udp_joint_voltage_.joint_voltage[i] = Udp_RM_Joint.joint_voltage[i];
        }
        if(arm_dof_g == 7)
        {
            udp_real_joint_.position[6] = Udp_RM_Joint.joint[6] * DEGREE_RAD;
            udp_joint_error_code_.joint_error[6] = Udp_RM_Joint.err_flag[6];
            udp_joint_current_.joint_current[6] = Udp_RM_Joint.joint_current[6];
            udp_joint_en_flag_.joint_en_flag[6] = Udp_RM_Joint.en_flag[6];
            udp_joint_speed_.joint_speed[6] = Udp_RM_Joint.joint_speed[6];
            udp_joint_temperature_.joint_temperature[6] = Udp_RM_Joint.joint_temperature[6];
            udp_joint_voltage_.joint_voltage[6] = Udp_RM_Joint.joint_voltage[6];
            udp_joint_error_code_.dof = 7;
        }
        if(udp_rm_err_.err_len != Udp_RM_Joint.udp_rm_err.err_len)
        {
            udp_rm_err_.err_len = Udp_RM_Joint.udp_rm_err.err_len;
            udp_rm_err_.err.resize(udp_rm_err_.err_len);
            for(int i = 0; i<udp_rm_err_.err_len; i++)
            {
                udp_rm_err_.err[i] = Udp_RM_Joint.udp_rm_err.err[i];
            }
        }
        this->Rm_Err_Result->publish(udp_rm_err_);
        udp_real_joint_.header.stamp = this->now();
        this->Joint_Position_Result->publish(udp_real_joint_);
        this->Joint_Error_Code_Result->publish(udp_joint_error_code_);
        udp_arm_current_status_.arm_current_status = Udp_RM_Joint.arm_current_status;
        //RCLCPP_INFO (this->get_logger(),"udp publisher arm_current_status is %d\n",udp_arm_current_status_.arm_current_status);
        this->Arm_Current_Status_Result->publish(udp_arm_current_status_);
        this->Joint_Current_Result->publish(udp_joint_current_);
        this->Joint_En_Flag_Result->publish(udp_joint_en_flag_);
        this->Joint_Speed_Result->publish(udp_joint_speed_);
        this->Joint_Temperature_Result->publish(udp_joint_temperature_);
        this->Joint_Voltage_Result->publish(udp_joint_voltage_);
        udp_arm_pose_.position.x = Udp_RM_Joint.joint_position[0];
        udp_arm_pose_.position.y = Udp_RM_Joint.joint_position[1];
        udp_arm_pose_.position.z = Udp_RM_Joint.joint_position[2];
        udp_arm_pose_.orientation.w = Udp_RM_Joint.joint_quat[0];
        udp_arm_pose_.orientation.x = Udp_RM_Joint.joint_quat[1];
        udp_arm_pose_.orientation.y = Udp_RM_Joint.joint_quat[2];
        udp_arm_pose_.orientation.z = Udp_RM_Joint.joint_quat[3];
        this->Arm_Position_Result->publish(udp_arm_pose_);
        udp_joint_pose_euler_.euler[0] = Udp_RM_Joint.joint_euler[0];
        udp_joint_pose_euler_.euler[1] = Udp_RM_Joint.joint_euler[1];
        udp_joint_pose_euler_.euler[2] = Udp_RM_Joint.joint_euler[2];
        udp_joint_pose_euler_.position[0]=Udp_RM_Joint.joint_position[0];
        udp_joint_pose_euler_.position[1]=Udp_RM_Joint.joint_position[1];
        udp_joint_pose_euler_.position[2]=Udp_RM_Joint.joint_position[2];
        this->Joint_Pose_Euler_Result->publish(udp_joint_pose_euler_);
        // sys_err_.data = Udp_RM_Joint.sys_err;
        // this->Sys_Err_Result->publish(sys_err_);
        // arm_err_.data = Udp_RM_Joint.arm_err;
        // this->Arm_Err_Result->publish(arm_err_);
        arm_coordinate_.data = Udp_RM_Joint.coordinate;
        this->Arm_Coordinate_Result->publish(arm_coordinate_);
        if(udp_hand_g == true)
        {
            for(int i = 0;i<6;i++)
            {
                udp_hand_status_.hand_angle[i] = Udp_RM_Joint.hand_angle[i];
                udp_hand_status_.hand_force[i] = Udp_RM_Joint.hand_force[i];
                udp_hand_status_.hand_pos[i] = Udp_RM_Joint.hand_pos[i];
                udp_hand_status_.hand_state[i] = Udp_RM_Joint.hand_state[i];
            }
            udp_hand_status_.hand_err = Udp_RM_Joint.hand_err;  
            this->Hand_Status_Result->publish(udp_hand_status_);
        }
        if(rm_plus_base_g == true)
        {
            for(int i = 0;i<10;i++)
            {
                udp_rm_plus_base_.pos_up[i] = Udp_RM_Joint.udp_rm_plus_base_info.pos_up[i];
                udp_rm_plus_base_.pos_low[i] = Udp_RM_Joint.udp_rm_plus_base_info.pos_low[i];
                udp_rm_plus_base_.speed_up[i] = Udp_RM_Joint.udp_rm_plus_base_info.speed_up[i];
                udp_rm_plus_base_.speed_low[i] = Udp_RM_Joint.udp_rm_plus_base_info.speed_low[i];
                udp_rm_plus_base_.force_up[i] = Udp_RM_Joint.udp_rm_plus_base_info.force_up[i];
                udp_rm_plus_base_.force_low[i] = Udp_RM_Joint.udp_rm_plus_base_info.force_low[i];
                udp_rm_plus_base_.angle_low[i] = Udp_RM_Joint.udp_rm_plus_base_info.angle_low[i];
                udp_rm_plus_base_.angle_up[i] = Udp_RM_Joint.udp_rm_plus_base_info.angle_up[i];
            }
            for(int i = 0; i < 2; i++)
            {
                udp_rm_plus_base_.pos_up[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.pos_up[10+i];
                udp_rm_plus_base_.pos_low[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.pos_low[10+i];
                udp_rm_plus_base_.speed_up[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.speed_up[10+i];
                udp_rm_plus_base_.speed_low[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.speed_low[10+i];
                udp_rm_plus_base_.force_up[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.force_up[10+i];
                udp_rm_plus_base_.force_low[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.force_low[10+i];
                udp_rm_plus_base_.angle_low[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.angle_low[10+i];
                udp_rm_plus_base_.angle_up[10+i] = Udp_RM_Joint.udp_rm_plus_base_info.angle_up[10+i];
            }
            udp_rm_plus_base_.manu = Udp_RM_Joint.udp_rm_plus_base_info.manu;
            udp_rm_plus_base_.hv = Udp_RM_Joint.udp_rm_plus_base_info.hv;
            udp_rm_plus_base_.sv = Udp_RM_Joint.udp_rm_plus_base_info.sv;
            udp_rm_plus_base_.bv = Udp_RM_Joint.udp_rm_plus_base_info.bv;

            udp_rm_plus_base_.id = Udp_RM_Joint.udp_rm_plus_base_info.id;
            udp_rm_plus_base_.dof = Udp_RM_Joint.udp_rm_plus_base_info.dof;
            udp_rm_plus_base_.check = Udp_RM_Joint.udp_rm_plus_base_info.check;
            udp_rm_plus_base_.bee = Udp_RM_Joint.udp_rm_plus_base_info.bee;
            udp_rm_plus_base_.force = Udp_RM_Joint.udp_rm_plus_base_info.force;
            udp_rm_plus_base_.touch = Udp_RM_Joint.udp_rm_plus_base_info.touch;
            udp_rm_plus_base_.touch_num = Udp_RM_Joint.udp_rm_plus_base_info.touch_num;
            udp_rm_plus_base_.touch_sw = Udp_RM_Joint.udp_rm_plus_base_info.touch_sw;
            udp_rm_plus_base_.hand = Udp_RM_Joint.udp_rm_plus_base_info.hand;
            this->Rm_Plus_Base_Result->publish(udp_rm_plus_base_);
        }
        if(rm_plus_state_g == true)
        {
            udp_rm_plus_state_.sys_state = Udp_RM_Joint.udp_rm_plus_state_info.sys_state;
            for(int i = 0; i < 12; i++)
            {
                udp_rm_plus_state_.dof_state[i] = Udp_RM_Joint.udp_rm_plus_state_info.dof_state[i];
                udp_rm_plus_state_.dof_err[i] = Udp_RM_Joint.udp_rm_plus_state_info.dof_err[i];
                udp_rm_plus_state_.pos[i] = Udp_RM_Joint.udp_rm_plus_state_info.pos[i];
                udp_rm_plus_state_.speed[i] = Udp_RM_Joint.udp_rm_plus_state_info.speed[i];
                udp_rm_plus_state_.angle[i] = Udp_RM_Joint.udp_rm_plus_state_info.angle[i];
                udp_rm_plus_state_.current[i] = Udp_RM_Joint.udp_rm_plus_state_info.current[i];
                udp_rm_plus_state_.normal_force[i] = Udp_RM_Joint.udp_rm_plus_state_info.normal_force[i];
                udp_rm_plus_state_.tangential_force[i] = Udp_RM_Joint.udp_rm_plus_state_info.tangential_force[i];
                udp_rm_plus_state_.tangential_force_dir[i] = Udp_RM_Joint.udp_rm_plus_state_info.tangential_force_dir[i];
                udp_rm_plus_state_.tsa[i] = Udp_RM_Joint.udp_rm_plus_state_info.tsa[i];
                udp_rm_plus_state_.tma[i] = Udp_RM_Joint.udp_rm_plus_state_info.tma[i];
                udp_rm_plus_state_.touch_data[i] = Udp_RM_Joint.udp_rm_plus_state_info.touch_data[i];
                udp_rm_plus_state_.force[i] = Udp_RM_Joint.udp_rm_plus_state_info.force[i];
            }
            for(int i = 12; i < 18; i++)
            {
                udp_rm_plus_state_.normal_force[i] = Udp_RM_Joint.udp_rm_plus_state_info.normal_force[i];
                udp_rm_plus_state_.tangential_force[i] = Udp_RM_Joint.udp_rm_plus_state_info.tangential_force[i];
                udp_rm_plus_state_.tangential_force_dir[i] = Udp_RM_Joint.udp_rm_plus_state_info.tangential_force_dir[i];
                udp_rm_plus_state_.touch_data[i] = Udp_RM_Joint.udp_rm_plus_state_info.touch_data[i];
            }
            this->Rm_Plus_State_Result->publish(udp_rm_plus_state_);
        }
        if(Udp_RM_Joint.control_version == 2)
        {
            udp_sixforce_.force_fx = Udp_RM_Joint.six_force[0];
            udp_sixforce_.force_fy = Udp_RM_Joint.six_force[1];
            udp_sixforce_.force_fz = Udp_RM_Joint.six_force[2];
            udp_sixforce_.force_mx = Udp_RM_Joint.six_force[3];
            udp_sixforce_.force_my = Udp_RM_Joint.six_force[4];
            udp_sixforce_.force_mz = Udp_RM_Joint.six_force[5];
            this->Six_Force_Result->publish(udp_sixforce_);
            udp_zeroforce_.force_fx = Udp_RM_Joint.zero_force[0];
            udp_zeroforce_.force_fy = Udp_RM_Joint.zero_force[1];
            udp_zeroforce_.force_fz = Udp_RM_Joint.zero_force[2];
            udp_zeroforce_.force_mx = Udp_RM_Joint.zero_force[3];
            udp_zeroforce_.force_my = Udp_RM_Joint.zero_force[4];
            udp_zeroforce_.force_mz = Udp_RM_Joint.zero_force[5];
            this->Six_Zero_Force_Result->publish(udp_zeroforce_);
        }
        if(Udp_RM_Joint.control_version == 3)
        {
            udp_oneforce_.force_fz = Udp_RM_Joint.one_force;
            this->One_Force_Result->publish(udp_oneforce_);
            udp_onezeroforce_.force_fz = Udp_RM_Joint.one_zero_force;
            this->One_Zero_Force_Result->publish(udp_onezeroforce_);
        }
        if(ctrl_flag == true )
        {
            rclcpp::shutdown();
        }
    }
    else
    {
        if(come_time == 0)
        {Arm_Close();}
        come_time++;
        while(Arm_Socket_Start_Connect())
        {
            if(ctrl_flag == true )
            {
                rclcpp::shutdown();
                exit(0);
            }
            RCLCPP_INFO (this->get_logger(),"Wait for connect ");
            sleep(1);
                    
        }
        Arm_Start();
        come_time = 0;
        connect_state = 0;
        RCLCPP_INFO (this->get_logger(),"Connect success\n");
    }
}

void UdpPublisherNode::heart_timer_callback()
{
    int run_mode;
    if(connect_state == 0)
    {
        connect_state = Rm_Api.rm_get_arm_run_mode(robot_handle, &run_mode);
        // RCLCPP_INFO (this->get_logger(),"connect_state = %d\n",connect_state);
    }
    else
    {
        ;
    }
}

bool UdpPublisherNode::read_data()
{
    memset(udp_socket_buffer, 0, sizeof(udp_socket_buffer));

    ssize_t numBytes = recvfrom(16, udp_socket_buffer, sizeof(udp_socket_buffer), 0,
        (struct sockaddr*) & clientAddr, &clientAddrLen);
    if (numBytes < 0) {
        // std::cerr << "Error in recvfrom" << std::endl;
        close(16);
        return false;
    }
    // 将接收到的数据输出到控制台
    udp_socket_buffer[numBytes] = '\0'; // 添加字符串结束符
    std::cout << "Received from "                                         //<< inet_ntoa(clientAddr.sin_addr)
    << ":" << ntohs(clientAddr.sin_port) << ": "
    << udp_socket_buffer << std::endl;
    if((udp_socket_buffer[numBytes-2]==0X0D)&&(udp_socket_buffer[numBytes-1]==0X0A))
    {
        return true;
    }
    else
    {
        // ROS_ERROR("udp_socket_buffer IS error");
        return false;
    }
}

UdpPublisherNode::UdpPublisherNode():
    rclcpp::Node("udp_publish_node"){
        /*************************************************多线程********************************************/
        callback_group_time1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_time2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_time3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        /*****************************************************UDP定时器*****************************************************************/
        Udp_Timer = this->create_wall_timer(std::chrono::milliseconds(udp_cycle_g), 
        std::bind(&UdpPublisherNode::udp_timer_callback,this), callback_group_time1_);
        /*****************************************************定时器*****************************************************************/
        Heart_Timer = this->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&UdpPublisherNode::heart_timer_callback,this), callback_group_time2_);
        /********************************************************************UDP传输数据**********************************************************/
        Joint_Position_Result = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);                                    //发布当前的关节角度
        Arm_Position_Result = this->create_publisher<geometry_msgs::msg::Pose>("rm_driver/udp_arm_position", 10);                            //发布当前的关节姿态
        Six_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/udp_six_force", 10);                          //发布当前的原始六维力数据
        Six_Zero_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/udp_six_zero_force", 10);                //发布当前标坐标系下六维力数据
        One_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/udp_one_force", 10);                          //发布当前的原始一维力数据
        One_Zero_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/udp_one_zero_force", 10);                //发布当前目标坐标系下一维力数据
        Joint_Error_Code_Result = this->create_publisher<rm_ros_interfaces::msg::Jointerrorcode>("rm_driver/udp_joint_error_code", 10);      //发布当前的关节错误码
        // Sys_Err_Result = this->create_publisher<std_msgs::msg::UInt16>("rm_driver/udp_sys_err", 10);                                      //发布当前的系统错误码
        Rm_Err_Result = this->create_publisher<rm_ros_interfaces::msg::Rmerr>("rm_driver/udp_rm_err", 10);                                   //发布当前的机械臂错误码
        Arm_Coordinate_Result = this->create_publisher<std_msgs::msg::UInt16>("rm_driver/udp_arm_coordinate", 10);                           //发布当前六维力数据的基准坐标系
        Hand_Status_Result = this->create_publisher<rm_ros_interfaces::msg::Handstatus>("rm_driver/udp_hand_status", 10);                    //发布灵巧手状态数据

        Arm_Current_Status_Result = this->create_publisher<rm_ros_interfaces::msg::Armcurrentstatus>("rm_driver/udp_arm_current_status", 10);
        Joint_Current_Result = this->create_publisher<rm_ros_interfaces::msg::Jointcurrent>("rm_driver/udp_joint_current",10);
        Joint_En_Flag_Result = this->create_publisher<rm_ros_interfaces::msg::Jointenflag>("rm_driver/udp_joint_en_flag",10);
        Joint_Pose_Euler_Result = this->create_publisher<rm_ros_interfaces::msg::Jointposeeuler>("rm_driver/udp_joint_pose_euler",10);
        Joint_Speed_Result = this->create_publisher<rm_ros_interfaces::msg::Jointspeed>("rm_driver/udp_joint_speed",10);
        Joint_Temperature_Result = this->create_publisher<rm_ros_interfaces::msg::Jointtemperature>("rm_driver/udp_joint_temperature",10);
        Joint_Voltage_Result = this->create_publisher<rm_ros_interfaces::msg::Jointvoltage>("rm_driver/udp_joint_voltage",10);
        Rm_Plus_Base_Result = this->create_publisher<rm_ros_interfaces::msg::Rmplusbase>("rm_driver/udp_rm_plus_base",10);
        Rm_Plus_State_Result = this->create_publisher<rm_ros_interfaces::msg::Rmplusstate>("rm_driver/udp_rm_plus_state",10);
    }


RmArm::~RmArm()
{ 
    Arm_Close();
}

RmArm::RmArm():
    rclcpp::Node("rm_driver"){
    //参数初始化
    this->declare_parameter("arm_ip", "192.168.1.188");
    arm_ip_ = this->get_parameter("arm_ip").as_string();
    
    this->declare_parameter("udp_ip", "192.168.1.10");
    udp_ip_ = this->get_parameter("udp_ip").as_string();

    this->declare_parameter<std::string>("arm_type", arm_type_);
    this->get_parameter<std::string>("arm_type", arm_type_);

    this->declare_parameter<int>("tcp_port", tcp_port_);
    this->get_parameter<int>("tcp_port", tcp_port_);

    this->declare_parameter<int>("udp_port", udp_port_);
    this->get_parameter<int>("udp_port", udp_port_);

    this->declare_parameter<int>("arm_dof", arm_dof_);
    this->get_parameter<int>("arm_dof", arm_dof_);

    this->declare_parameter<int>("udp_cycle", udp_cycle_);
    this->get_parameter<int>("udp_cycle", udp_cycle_);

    this->declare_parameter<int>("udp_force_coordinate", udp_force_coordinate_);
    this->get_parameter<int>("udp_force_coordinate", udp_force_coordinate_);

    this->declare_parameter<bool>("udp_hand", udp_hand_);
    this->get_parameter<bool>("udp_hand", udp_hand_);

    this->declare_parameter<bool>("udp_plus_base", udp_rm_plus_base_);
    this->get_parameter<bool>("udp_plus_base", udp_rm_plus_base_);

    this->declare_parameter<bool>("udp_plus_state", udp_rm_plus_state_);
    this->get_parameter<bool>("udp_plus_state", udp_rm_plus_state_);

    this->declare_parameter<int>("trajectory_mode", trajectory_mode_);
    this->get_parameter<int>("trajectory_mode", trajectory_mode_);

    this->declare_parameter<int>("radio", radio_);
    this->get_parameter<int>("radio", radio_);

    this->declare_parameter("arm_joints", arm_joints);
    
    udp_cycle_g = udp_cycle_;
    if(arm_type_ == "RM_65")
    {
        // Rm_Api.Service_RM_API_Init(65, NULL);
        // RCLCPP_INFO (this->get_logger(),"0000000000000000000000000000000000000000000000000");
        realman_arm = 65;
    }
    else if(arm_type_ == "RM_75")
    {
        // Rm_Api.Service_RM_API_Init(75, NULL);
        realman_arm = 75;
    }
    else if(arm_type_ == "RM_63")
    {
        // Rm_Api.Service_RM_API_Init(632, NULL);
        realman_arm = 63;
    }
    else if(arm_type_ == "RM_eco65")
    {
        // Rm_Api.Service_RM_API_Init(651, NULL);
        realman_arm = 651;
    }
    else if(arm_type_ == "RM_eco63")
    {
        // Rm_Api.Service_RM_API_Init(634, NULL);
        realman_arm = 634;
    }
    else if(arm_type_ == "RM_eco62")
    {
        // Rm_Api.Service_RM_API_Init(62, NULL);
        realman_arm = 62;
    }
    else if(arm_type_ == "GEN_72")
    {
        // Rm_Api.Service_RM_API_Init(72, NULL);
        realman_arm = 72;
    }
    tcp_ip = (char*)arm_ip_.c_str();
    
    tcp_port = tcp_port_;
    udp_hand_g = udp_hand_;
    rm_plus_base_g = udp_rm_plus_base_;
    rm_plus_state_g = udp_rm_plus_state_;
    RCLCPP_INFO (this->get_logger(),"arm_ip is %s", arm_ip_.c_str());
    while(Arm_Socket_Start_Connect())
    {
        if(ctrl_flag == true )
        {
            rclcpp::shutdown();
            exit(0);
        }
        RCLCPP_INFO (this->get_logger(),"Waiting for connect");
        sleep(1);
    }
    usleep(2000000);
    RCLCPP_INFO (this->get_logger(),"%s_driver is running ",arm_type_.c_str());
    /************************************************初始化变量********************************************/
    udp_real_joint_.name.resize(arm_dof_);
    udp_real_joint_.position.resize(arm_dof_);
    udp_joint_error_code_.joint_error.resize(arm_dof_);
    Arm_original_state.joint.resize(arm_dof_);
    Arm_state.joint.resize(arm_dof_);
    udp_joint_current_.joint_current.resize(arm_dof_);
    udp_joint_en_flag_.joint_en_flag.resize(arm_dof_);
    udp_joint_speed_.joint_speed.resize(arm_dof_);
    udp_joint_temperature_.joint_temperature.resize(arm_dof_);
    udp_joint_voltage_.joint_voltage.resize(arm_dof_);
    arm_dof_g = arm_dof_;
    if (this->get_parameter("arm_joints", arm_joints))
    {
        for (int i = 0; i < arm_dof_; i++)
        {
            udp_real_joint_.name[i] = arm_joints[i];
            // RCLCPP_INFO (this->get_logger(),"arm_joints[%d]: %s",  i,  arm_joints[i].c_str());
        }
    }
    /**************************************************end**********************************************/
    
    /**********************************************初始化、连接函数****************************************/

    Arm_Start();

    /***************************************************end**********************************************/

    /*************************************************多线程********************************************/
    callback_group_sub1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt1 = rclcpp::SubscriptionOptions();
    sub_opt1.callback_group = callback_group_sub1_;
    callback_group_sub2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt2 = rclcpp::SubscriptionOptions();
    sub_opt2.callback_group = callback_group_sub2_;
    callback_group_sub3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt3 = rclcpp::SubscriptionOptions();
    sub_opt3.callback_group = callback_group_sub3_;
    callback_group_sub4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt4 = rclcpp::SubscriptionOptions();
    sub_opt4.callback_group = callback_group_sub4_;
    
    Get_Arm_Version();//获取机械臂版本

    Set_UDP_Configuration(udp_cycle_, udp_port_, udp_force_coordinate_, udp_ip_, udp_hand_, udp_rm_plus_base_, udp_rm_plus_state_);

    /******************************************************获取udp配置********************************************************************/
    Get_Realtime_Push_Result = this->create_publisher<rm_ros_interfaces::msg::Setrealtimepush>("rm_driver/get_realtime_push_result", rclcpp::ParametersQoS());
    Get_Realtime_Push_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_realtime_push_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Realtime_Push_Callback,this,std::placeholders::_1),
        sub_opt2);
    /******************************************************设置udp配置********************************************************************/
    Set_Realtime_Push_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_realtime_push_result", rclcpp::ParametersQoS());
    Set_Realtime_Push_Cmd = this->create_subscription<rm_ros_interfaces::msg::Setrealtimepush>("rm_driver/set_realtime_push_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Realtime_Push_Callback,this,std::placeholders::_1),
        sub_opt2);
/******************************************************************************end*******************************************************************/

/***********************************************************************运动配置**********************************************************************/
    /****************************************MoveJ运动控制*************************************/
    MoveJ_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/movej_result", rclcpp::ParametersQoS());
    MoveJ_Cmd = this->create_subscription<rm_ros_interfaces::msg::Movej>("rm_driver/movej_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_MoveJ_Callback,this,std::placeholders::_1),
        sub_opt4);
    /****************************************MoveL运动控制*************************************/
    MoveL_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/movel_result", rclcpp::ParametersQoS());
    MoveL_Cmd = this->create_subscription<rm_ros_interfaces::msg::Movel>("rm_driver/movel_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_MoveL_Callback,this,std::placeholders::_1),
        sub_opt4);
    /****************************************MoveC运动控制*************************************/
    MoveC_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/movec_result", rclcpp::ParametersQoS());
    MoveC_Cmd = this->create_subscription<rm_ros_interfaces::msg::Movec>("rm_driver/movec_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_MoveC_Callback,this,std::placeholders::_1),
        sub_opt4);
    /******************************************角度透传*****************************************/
    Movej_CANFD_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointpos>("rm_driver/movej_canfd_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Movej_CANFD_Callback,this,std::placeholders::_1),
        sub_opt4);
    Movej_CANFD_Custom_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointposcustom>("rm_driver/movej_canfd_custom_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Movej_CANFD_Custom_Callback,this,std::placeholders::_1),
        sub_opt4);
    /*******************************************位姿透传****************************************/
    Movep_CANFD_Cmd = this->create_subscription<rm_ros_interfaces::msg::Cartepos>("rm_driver/movep_canfd_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Movep_CANFD_Callback,this,std::placeholders::_1),
        sub_opt4);
    Movep_CANFD_Custom_Cmd = this->create_subscription<rm_ros_interfaces::msg::Carteposcustom>("rm_driver/movep_canfd_custom_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Movep_CANFD_Custom_Callback,this,std::placeholders::_1),
        sub_opt4);
    /****************************************MoveJ_P运动控制*************************************/
    MoveJ_P_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/movej_p_result", rclcpp::ParametersQoS());
    MoveJ_P_Cmd = this->create_subscription<rm_ros_interfaces::msg::Movejp>("rm_driver/movej_p_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_MoveJ_P_Callback,this,std::placeholders::_1),
        sub_opt4);
    /***********************************************轨迹急停****************************************/
    Move_Stop_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/move_stop_result", rclcpp::ParametersQoS());
    Move_Stop_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/move_stop_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Move_Stop_Callback,this,std::placeholders::_1),
        sub_opt2);
/******************************************************************************end*******************************************************************/

/******************************************************************************示教指令*****************************************************************/
    /*********************************************************关节示教*****************************************************************/
    Set_Joint_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_joint_teach_result", rclcpp::ParametersQoS());
    Set_Joint_Teach_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointteach>("rm_driver/set_joint_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Joint_Teach_Callback,this,std::placeholders::_1),
        sub_opt4);
    /*********************************************************位置示教*****************************************************************/
    Set_Pos_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_pos_teach_result", rclcpp::ParametersQoS());
    Set_Pos_Teach_Cmd = this->create_subscription<rm_ros_interfaces::msg::Posteach>("rm_driver/set_pos_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Pos_Teach_Callback,this,std::placeholders::_1),
        sub_opt4);
    /*********************************************************姿态示教*****************************************************************/
    Set_Ort_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_ort_teach_result", rclcpp::ParametersQoS());
    Set_Ort_Teach_Cmd = this->create_subscription<rm_ros_interfaces::msg::Ortteach>("rm_driver/set_ort_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Ort_Teach_Callback,this,std::placeholders::_1),
        sub_opt4);
    /*********************************************************示教停止*****************************************************************/
    Set_Stop_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_stop_teach_result", rclcpp::ParametersQoS());
    Set_Stop_Teach_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/set_stop_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Stop_Teach_Callback,this,std::placeholders::_1),
        sub_opt2);
/******************************************************************************end*****************************************************************/

    /************************************************************************查询机械臂固件版本***************************************************************/
    // Get_Arm_Software_Version_Result = this->create_publisher<rm_ros_interfaces::msg::Armsoftversion>("rm_driver/get_arm_software_version_result", rclcpp::ParametersQoS());
    // Get_Arm_Software_Version_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_arm_software_version_cmd",rclcpp::ParametersQoS(),
    //     std::bind(&RmArm::Arm_Get_Arm_Software_Version_Callback,this,std::placeholders::_1),
    //     sub_opt2);
    /*********************************************************************************end*******************************************************************/

/**********************************************************************透传力位混合控制***********************************************************/
    /******************************************************开启力位混合********************************************************************/
    Start_Force_Position_Move_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/start_force_position_move_result", rclcpp::ParametersQoS());
    Start_Force_Position_Move_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/start_force_position_move_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Start_Force_Position_Move_Callback,this,std::placeholders::_1),
        sub_opt2);
    /********************************************************关闭力位混合*******************************************************************/
    Stop_Force_Position_Move_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/stop_force_position_move_result", rclcpp::ParametersQoS());
    Stop_Force_Position_Move_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/stop_force_position_move_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Stop_Force_Position_Move_Callback,this,std::placeholders::_1),
        sub_opt2);
    /********************************************************角度透传力位混合*****************************************************************/
    Force_Position_Move_Joint_Cmd = this->create_subscription<rm_ros_interfaces::msg::Forcepositionmovejoint>("rm_driver/force_position_move_joint_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Force_Position_Move_Joint_Callback,this,std::placeholders::_1),
        sub_opt4);
    /********************************************************位姿透传力位混合*****************************************************************/
    Force_Position_Move_Pose_Cmd = this->create_subscription<rm_ros_interfaces::msg::Forcepositionmovepose>("rm_driver/force_position_move_pose_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Force_Position_Move_Pose_Callback,this,std::placeholders::_1),
        sub_opt4);
    /********************************************************设置力位混合控制*******************************************************************/
    Set_Force_Postion_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_force_postion_result", rclcpp::ParametersQoS());
    Set_Force_Postion_Cmd = this->create_subscription<rm_ros_interfaces::msg::Setforceposition>("rm_driver/set_force_postion_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Force_Postion_Callback,this,std::placeholders::_1),
        sub_opt4);
    /********************************************************结束力位混合控制*******************************************************************/
    Stop_Force_Postion_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/stop_force_postion_result", rclcpp::ParametersQoS());
    Stop_Force_Postion_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/stop_force_postion_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Stop_Force_Postion_Callback,this,std::placeholders::_1),
        sub_opt4);
/****************************************************************************end******************************************************************/

/************************************************************************坐标系指令*************************************************************/
    /**********************************************************切换工作坐标系********************************************************************/
    Change_Work_Frame_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/change_work_frame_result", rclcpp::ParametersQoS());
    Change_Work_Frame_Cmd = this->create_subscription<std_msgs::msg::String>("rm_driver/change_work_frame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Change_Work_Frame_Callback,this,std::placeholders::_1),
        sub_opt2);
    /**********************************************************获得工作坐标系********************************************************************/
    Get_Curr_WorkFrame_Result = this->create_publisher<std_msgs::msg::String>("rm_driver/get_curr_workFrame_result", rclcpp::ParametersQoS());
    Get_Curr_WorkFrame_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_curr_workFrame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Curr_WorkFrame_Callback,this,std::placeholders::_1),
        sub_opt2);
    /**********************************************************获得工具坐标系********************************************************************/
    Get_Current_Tool_Frame_Result = this->create_publisher<std_msgs::msg::String>("rm_driver/get_current_tool_frame_result", rclcpp::ParametersQoS());
    Get_Current_Tool_Frame_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_current_tool_frame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Current_Tool_Frame_Callback,this,std::placeholders::_1),
        sub_opt2);
    /**********************************************************获得所有工具坐标系********************************************************************/
    Get_All_Tool_Frame_Result = this->create_publisher<rm_ros_interfaces::msg::Getallframe>("rm_driver/get_all_tool_frame_result", rclcpp::ParametersQoS());
    Get_All_Tool_Frame_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_all_tool_frame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_All_Tool_Frame_Callback,this,std::placeholders::_1),
        sub_opt2);
    /**********************************************************获得所有工作坐标系********************************************************************/
    Get_All_Work_Frame_Result = this->create_publisher<rm_ros_interfaces::msg::Getallframe>("rm_driver/get_all_work_frame_result", rclcpp::ParametersQoS());
    Get_All_Work_Frame_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_all_work_frame_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_All_Work_Frame_Callback,this,std::placeholders::_1),
        sub_opt2);
/*****************************************************************************end***************************************************************/
    
    /**********************************************************设置工具端电源输出********************************************************************/
    Set_Tool_Voltage_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_tool_voltage_result", rclcpp::ParametersQoS());
    Set_Tool_Voltage_Cmd = this->create_subscription<std_msgs::msg::UInt16>("rm_driver/set_tool_voltage_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Tool_Voltage_Callback,this,std::placeholders::_1),
        sub_opt2);
    /*****************************************************************************end***************************************************************/

    /**********************************************************清除机械臂错误码********************************************************************/
    Set_Joint_Err_Clear_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_joint_err_clear_result", rclcpp::ParametersQoS());
    Set_Joint_Err_Clear_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointerrclear>("rm_driver/set_joint_err_clear_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Joint_Err_Clear_Callback,this,std::placeholders::_1),
        sub_opt2);
    /*****************************************************************************end***************************************************************/

/********************************************************************末端工具-手爪控制****************************************************************/
    /****************************************手爪持续力控夹取**********************************/
    Set_Gripper_Pick_On_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_gripper_pick_on_result", rclcpp::ParametersQoS());
    Set_Gripper_Pick_On_Cmd = this->create_subscription<rm_ros_interfaces::msg::Gripperpick>("rm_driver/set_gripper_pick_on_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Gripper_Pick_On_Callback,this,std::placeholders::_1),
        sub_opt3);
    /********************************************手爪力控夹取**********************************/
    Set_Gripper_Pick_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_gripper_pick_result", rclcpp::ParametersQoS());
    Set_Gripper_Pick_Cmd = this->create_subscription<rm_ros_interfaces::msg::Gripperpick>("rm_driver/set_gripper_pick_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Gripper_Pick_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*****************************************手爪到达指定位置**********************************/
    Set_Gripper_Position_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_gripper_position_result", rclcpp::ParametersQoS());
    Set_Gripper_Position_Cmd = this->create_subscription<rm_ros_interfaces::msg::Gripperset>("rm_driver/set_gripper_position_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Gripper_Position_Callback,this,std::placeholders::_1),
        sub_opt3);
/*******************************************************************************end*****************************************************************/

/********************************************************************末端工具-五指灵巧手控制************************************************************/
    /****************************************设置灵巧手手势序号**********************************/
    Set_Hand_Posture_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_posture_result", rclcpp::ParametersQoS());
    Set_Hand_Posture_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handposture>("rm_driver/set_hand_posture_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Posture_Callback,this,std::placeholders::_1),
        sub_opt3);
    /***************************************设置灵巧手动作序列序号*********************************/
    Set_Hand_Seq_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_seq_result", rclcpp::ParametersQoS());
    Set_Hand_Seq_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handseq>("rm_driver/set_hand_seq_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Seq_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手角度************************************/
    Set_Hand_Angle_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_angle_result", rclcpp::ParametersQoS());
    Set_Hand_Angle_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handangle>("rm_driver/set_hand_angle_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Angle_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手速度************************************/
    Set_Hand_Speed_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_speed_result", rclcpp::ParametersQoS());
    Set_Hand_Speed_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handspeed>("rm_driver/set_hand_speed_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Speed_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手力度************************************/
    Set_Hand_Force_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_force_result", rclcpp::ParametersQoS());
    Set_Hand_Force_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handforce>("rm_driver/set_hand_force_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Force_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手角度跟随************************************/
    Set_Hand_Follow_Angle_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_follow_angle_result", rclcpp::ParametersQoS());
    Set_Hand_Follow_Angle_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handangle>("rm_driver/set_hand_follow_angle_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Follow_Angle_Callback,this,std::placeholders::_1),
        sub_opt3);
    /*******************************************设置灵巧手姿势跟随************************************/
    Set_Hand_Follow_Pos_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_hand_follow_pos_result", rclcpp::ParametersQoS());
    Set_Hand_Follow_Pos_Cmd = this->create_subscription<rm_ros_interfaces::msg::Handangle>("rm_driver/set_hand_follow_pos_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Hand_Follow_Pos_Callback,this,std::placeholders::_1),
        sub_opt3);
/*******************************************************************************end*****************************************************************/

/********************************************************************升降机构************************************************************/
    /****************************************设置升降机构速度**********************************/
    Set_Lift_Speed_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_lift_speed_result", rclcpp::ParametersQoS());
    Set_Lift_Speed_Cmd = this->create_subscription<rm_ros_interfaces::msg::Liftspeed>("rm_driver/set_lift_speed_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Lift_Speed_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************设置升降机构高度**********************************/
    Set_Lift_Height_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_lift_height_result", rclcpp::ParametersQoS());
    Set_Lift_Height_Cmd = this->create_subscription<rm_ros_interfaces::msg::Liftheight>("rm_driver/set_lift_height_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Lift_Height_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************获取升降机构状态**********************************/
    Get_Lift_State_Result = this->create_publisher<rm_ros_interfaces::msg::Liftstate>("rm_driver/get_lift_state_result", rclcpp::ParametersQoS());
    Get_Lift_State_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_lift_state_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Lift_State_Callback,this,std::placeholders::_1),
        sub_opt3);
/*******************************************************************************end*****************************************************************/

    /***************************************************获取机械臂当前状态********************************************/
    Get_Current_Arm_Original_State_Result = this->create_publisher<rm_ros_interfaces::msg::Armoriginalstate>("rm_driver/get_current_arm_original_state_result", rclcpp::ParametersQoS());
    Get_Current_Arm_State_Result = this->create_publisher<rm_ros_interfaces::msg::Armstate>("rm_driver/get_current_arm_state_result", rclcpp::ParametersQoS());
    Get_Current_Arm_State_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_current_arm_state_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Current_Arm_State_Callback,this,std::placeholders::_1),
        sub_opt2);
/*********************************************************************六维力***************************************************************/
    /*****************************************************六维力数据清零**********************************************/
    Clear_Force_Data_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/clear_force_data_result", rclcpp::ParametersQoS());
    Clear_Force_Data_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/clear_force_data_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Clear_Force_Data_Callback,this,std::placeholders::_1),
        sub_opt2);
    /******************************************************获取六维力数据************************************************/
    /***************************************************传感器受到的外力数据***********************************************/
    Get_Force_Data_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_force_data_result", rclcpp::ParametersQoS());
    /***************************************************系统受到的外力数据***********************************************/
    Get_Zero_Force_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_zero_force_data_result", rclcpp::ParametersQoS());
    /************************************************工作坐标系下系统受到的外力数据******************************************/
    Get_Work_Zero_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_work_force_data_result", rclcpp::ParametersQoS());
    /***********************************************工具坐标系下系统受到的外力数据********************************************/
    Get_Tool_Zero_Result = this->create_publisher<rm_ros_interfaces::msg::Sixforce>("rm_driver/get_tool_force_data_result", rclcpp::ParametersQoS());
    Get_Force_Data_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_force_data_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Force_Data_Callback,this,std::placeholders::_1),
        sub_opt2);
/*******************************************************************************end*****************************************************************/

/********************************************************************末端生态协议************************************************************/
    /****************************************设置末端生态协议模式**********************************/
    Set_Rm_Plus_Mode_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_rm_plus_mode_result", rclcpp::ParametersQoS());
    Set_Rm_Plus_Mode_Cmd = this->create_subscription<std_msgs::msg::Int32>("rm_driver/set_rm_plus_mode_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Rm_Plus_Mode_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************查询末端生态协议模式**********************************/
    Get_Rm_Plus_Mode_Result = this->create_publisher<std_msgs::msg::Int32>("rm_driver/get_rm_plus_mode_result", rclcpp::ParametersQoS());
    Get_Rm_Plus_Mode_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_rm_plus_mode_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Rm_Plus_Mode_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************设置触觉传感器模式**********************************/
    Set_Rm_Plus_Touch_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_rm_plus_touch_result", rclcpp::ParametersQoS());
    Set_Rm_Plus_Touch_Cmd = this->create_subscription<std_msgs::msg::Int32>("rm_driver/set_rm_plus_touch_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Set_Rm_Plus_Touch_Callback,this,std::placeholders::_1),
        sub_opt3);
    /****************************************获取触觉传感器模式**********************************/
    Get_Rm_Plus_Touch_Result = this->create_publisher<std_msgs::msg::Int32>("rm_driver/get_rm_plus_touch_result", rclcpp::ParametersQoS());
    Get_Rm_Plus_Touch_Cmd = this->create_subscription<std_msgs::msg::Empty>("rm_driver/get_rm_plus_touch_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Arm_Get_Rm_Plus_Touch_Callback,this,std::placeholders::_1),
        sub_opt3);
/*******************************************************************************end*****************************************************************/
}   

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, my_handler); 
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),8,true);
    auto node = std::make_shared<RmArm>();
    auto udpnode = std::make_shared<UdpPublisherNode>();
    executor.add_node(node);
    executor.add_node(udpnode);
    executor.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}