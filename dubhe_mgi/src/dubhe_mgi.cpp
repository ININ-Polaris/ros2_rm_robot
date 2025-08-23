#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("mgi_current_pose_publisher", opts);

  const auto planning_group = node->get_parameter_or<std::string>("planning_group", "manipulator");
  const auto reference_frame = node->get_parameter_or<std::string>("reference_frame", "base_link");
  const auto ns = node->get_parameter_or<std::string>("robot_ns", "");

  // 发布当前末端位姿的 publisher
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

  // MoveGroup 接口初始化
  moveit::planning_interface::MoveGroupInterface mgi(node, planning_group);

  mgi.setPoseReferenceFrame(reference_frame);

  RCLCPP_INFO(node->get_logger(), "开始循环发布末端位姿...");

  rclcpp::Rate loop_rate(10); // 10Hz
  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped current_pose = mgi.getCurrentPose();
    pose_pub->publish(current_pose);
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "当前位姿: (%.3f, %.3f, %.3f)",
                         current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
