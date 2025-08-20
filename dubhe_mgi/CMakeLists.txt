cmake_minimum_required(VERSION 3.8)
project(dubhe_mgi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- 接口生成 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPose.srv"
  "srv/GetJointAngles.srv"
  "srv/PlanMotion.srv"
  "srv/ExecutePlan.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# ---- 可执行文件 ----
add_executable(dubhe_mgi_node src/dubhe_mgi.cpp)
ament_target_dependencies(dubhe_mgi_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
)

# ✅ 用 typesupport 目标来建立依赖和链接
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- 安装 ----
install(TARGETS dubhe_mgi_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

target_link_libraries(dubhe_mgi_node ${cpp_typesupport_target})

target_include_directories(dubhe_mgi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}  
)

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
