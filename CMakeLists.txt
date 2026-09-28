cmake_minimum_required(VERSION 3.8)
project(openarm_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(mtc_node
  src/mtc_node.cpp
  src/grasp.cpp
  src/tasks/pick_task.cpp
)

target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mtc_node
  rclcpp
  tf2
  geometry_msgs
  moveit_msgs
  moveit_task_constructor_core
  moveit_ros_planning_interface
  shape_msgs
)

add_executable(test_tcp_pose
  src/test_tcp_pose.cpp
)

ament_target_dependencies(test_tcp_pose
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

target_compile_features(mtc_node PUBLIC cxx_std_17)

install(
  TARGETS mtc_node test_tcp_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()