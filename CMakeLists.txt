cmake_minimum_required(VERSION 3.8)
project(moveit_joint_stepper_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(angles REQUIRED)

add_executable(move_joint_delta src/move_joint_delta.cpp)
ament_target_dependencies(move_joint_delta
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  angles
)

install(TARGETS move_joint_delta
  DESTINATION lib/${PROJECT_NAME})


add_executable(move_pose_target src/move_pose_target.cpp)
ament_target_dependencies(move_pose_target
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  angles
)
install(TARGETS move_pose_target
  DESTINATION lib/${PROJECT_NAME})

add_executable(snap_move_and_snap src/snap_move_and_snap.cpp)
ament_target_dependencies(snap_move_and_snap rclcpp)
install(TARGETS snap_move_and_snap DESTINATION lib/${PROJECT_NAME})


ament_package()

