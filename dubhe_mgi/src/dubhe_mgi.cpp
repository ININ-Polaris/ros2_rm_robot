#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

// 自定义服务消息（需要在单独的.srv文件中定义）
#include "dubhe_mgi/srv/execute_plan.hpp"
#include "dubhe_mgi/srv/get_joint_angles.hpp"
#include "dubhe_mgi/srv/get_pose.hpp"
#include "dubhe_mgi/srv/plan_motion.hpp"

class MoveItServiceNode : public rclcpp::Node {
public:
  MoveItServiceNode() : Node("moveit_service_node") {
    // 自动声明参数
    this->declare_parameter("planning_group", "manipulator");
    this->declare_parameter("reference_frame", "base_link");
    this->declare_parameter("robot_ns", "");

    // 获取参数
    planning_group_ = this->get_parameter("planning_group").as_string();
    reference_frame_ = this->get_parameter("reference_frame").as_string();
    robot_ns_ = this->get_parameter("robot_ns").as_string();

    RCLCPP_INFO(this->get_logger(), "Planning Group: %s",
                planning_group_.c_str());
    RCLCPP_INFO(this->get_logger(), "Reference Frame: %s",
                reference_frame_.c_str());

    // 初始化MoveGroupInterface
    initializeMoveGroup();

    // 创建服务
    createServices();

    RCLCPP_INFO(this->get_logger(), "MoveIt服务节点已启动");
  }

private:
  // 成员变量
  std::string planning_group_;
  std::string reference_frame_;
  std::string robot_ns_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;

  // 服务
  rclcpp::Service<robot_control_interfaces::srv::GetPose>::SharedPtr
      get_pose_service_;
  rclcpp::Service<robot_control_interfaces::srv::GetJointAngles>::SharedPtr
      get_joint_angles_service_;
  rclcpp::Service<robot_control_interfaces::srv::PlanMotion>::SharedPtr
      plan_motion_service_;
  rclcpp::Service<robot_control_interfaces::srv::ExecutePlan>::SharedPtr
      execute_plan_service_;

  void initializeMoveGroup() {
    moveit::planning_interface::MoveGroupInterface::Options mgi_opt(
        planning_group_, "robot_description", robot_ns_);

    mgi_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), mgi_opt, std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(5));

    mgi_->setPoseReferenceFrame(reference_frame_);
    mgi_->setPlanningTime(10.0);
    mgi_->setMaxVelocityScalingFactor(0.3);
    mgi_->setMaxAccelerationScalingFactor(0.3);
  }

  void createServices() {
    // 获取末端位姿服务
    get_pose_service_ =
        this->create_service<robot_control_interfaces::srv::GetPose>(
            "get_end_effector_pose",
            std::bind(&MoveItServiceNode::getPoseCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    // 获取关节角服务
    get_joint_angles_service_ =
        this->create_service<robot_control_interfaces::srv::GetJointAngles>(
            "get_joint_angles",
            std::bind(&MoveItServiceNode::getJointAnglesCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    // 运动规划服务
    plan_motion_service_ =
        this->create_service<robot_control_interfaces::srv::PlanMotion>(
            "plan_motion",
            std::bind(&MoveItServiceNode::planMotionCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    // 执行规划服务
    execute_plan_service_ =
        this->create_service<robot_control_interfaces::srv::ExecutePlan>(
            "execute_plan",
            std::bind(&MoveItServiceNode::executePlanCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
  }

  // 获取末端位姿回调函数
  void getPoseCallback(
      const std::shared_ptr<robot_control_interfaces::srv::GetPose::Request>
          request,
      std::shared_ptr<robot_control_interfaces::srv::GetPose::Response>
          response) {

    try {
      geometry_msgs::msg::PoseStamped current_pose = mgi_->getCurrentPose();
      response->current_pose = current_pose;
      response->success = true;
      response->message = "成功获取末端位姿";

      RCLCPP_INFO(this->get_logger(), "获取末端位姿: x=%.3f, y=%.3f, z=%.3f",
                  current_pose.pose.position.x, current_pose.pose.position.y,
                  current_pose.pose.position.z);
    } catch (const std::exception &e) {
      response->success = false;
      response->message = "获取末端位姿失败: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  // 获取关节角回调函数
  void getJointAnglesCallback(
      const std::shared_ptr<
          robot_control_interfaces::srv::GetJointAngles::Request>
          request,
      std::shared_ptr<robot_control_interfaces::srv::GetJointAngles::Response>
          response) {

    try {
      std::vector<double> joint_values = mgi_->getCurrentJointValues();
      std::vector<std::string> joint_names = mgi_->getJointNames();

      response->joint_names = joint_names;
      response->joint_angles = joint_values;
      response->success = true;
      response->message = "成功获取关节角度";

      RCLCPP_INFO(this->get_logger(), "获取关节角度，共%zu个关节",
                  joint_values.size());
      for (size_t i = 0; i < joint_names.size() && i < joint_values.size();
           ++i) {
        RCLCPP_INFO(this->get_logger(), "%s: %.3f rad", joint_names[i].c_str(),
                    joint_values[i]);
      }
    } catch (const std::exception &e) {
      response->success = false;
      response->message = "获取关节角度失败: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  // 运动规划回调函数
  void planMotionCallback(
      const std::shared_ptr<robot_control_interfaces::srv::PlanMotion::Request>
          request,
      std::shared_ptr<robot_control_interfaces::srv::PlanMotion::Response>
          response) {

    try {
      // 设置规划器
      std::string planner = request->planner_type;
      if (planner == "LIN") {
        mgi_->setPlannerId("LIN");
        RCLCPP_INFO(this->get_logger(), "使用Pilz LIN规划器");
      } else if (planner == "PTP") {
        mgi_->setPlannerId("PTP");
        RCLCPP_INFO(this->get_logger(), "使用Pilz PTP规划器");
      } else if (planner == "CHOMP") {
        mgi_->setPlannerId("CHOMPkConfigDefault");
        RCLCPP_INFO(this->get_logger(), "使用CHOMP规划器");
      } else {
        mgi_->setPlannerId("RRTConnect"); // 默认规划器
        RCLCPP_INFO(this->get_logger(), "使用默认RRTConnect规划器");
      }

      // 设置目标位姿或关节角度
      if (request->use_pose_target) {
        mgi_->setPoseTarget(request->target_pose);
        RCLCPP_INFO(this->get_logger(), "设置位姿目标: x=%.3f, y=%.3f, z=%.3f",
                    request->target_pose.pose.position.x,
                    request->target_pose.pose.position.y,
                    request->target_pose.pose.position.z);
      } else {
        mgi_->setJointValueTarget(request->target_joint_angles);
        RCLCPP_INFO(this->get_logger(), "设置关节角度目标");
      }

      // 设置速度和加速度缩放因子
      if (request->velocity_scaling > 0.0 && request->velocity_scaling <= 1.0) {
        mgi_->setMaxVelocityScalingFactor(request->velocity_scaling);
      }
      if (request->acceleration_scaling > 0.0 &&
          request->acceleration_scaling <= 1.0) {
        mgi_->setMaxAccelerationScalingFactor(request->acceleration_scaling);
      }

      // 执行规划
      RCLCPP_INFO(this->get_logger(), "开始规划...");
      moveit::core::MoveItErrorCode result = mgi_->plan(current_plan_);

      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        response->success = true;
        response->message = "规划成功完成";
        response->planning_time = current_plan_.planning_time_;
        RCLCPP_INFO(this->get_logger(), "规划成功，用时%.3f秒",
                    current_plan_.planning_time_);
      } else {
        response->success = false;
        response->message = "规划失败: " + std::to_string(result.val);
        RCLCPP_ERROR(this->get_logger(), "规划失败，错误代码: %d", result.val);
      }

      // 清除目标
      mgi_->clearPoseTargets();

    } catch (const std::exception &e) {
      response->success = false;
      response->message = "规划过程中发生异常: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  // 执行规划回调函数
  void executePlanCallback(
      const std::shared_ptr<robot_control_interfaces::srv::ExecutePlan::Request>
          request,
      std::shared_ptr<robot_control_interfaces::srv::ExecutePlan::Response>
          response) {

    try {
      if (current_plan_.trajectory_.joint_trajectory.points.empty()) {
        response->success = false;
        response->message = "没有可执行的规划，请先进行规划";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "开始执行规划...");
      moveit::core::MoveItErrorCode result = mgi_->execute(current_plan_);

      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        response->success = true;
        response->message = "规划执行成功";
        RCLCPP_INFO(this->get_logger(), "规划执行成功");
      } else {
        response->success = false;
        response->message = "规划执行失败: " + std::to_string(result.val);
        RCLCPP_ERROR(this->get_logger(), "规划执行失败，错误代码: %d",
                     result.val);
      }

      // 停止运动
      mgi_->stop();

    } catch (const std::exception &e) {
      response->success = false;
      response->message = "执行过程中发生异常: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveItServiceNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}