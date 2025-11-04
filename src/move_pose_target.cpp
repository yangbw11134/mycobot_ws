#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>

struct Options {
  std::string group = "arm_group"; // 네 MoveIt 그룹명
  double x = 0.2;  // m
  double y = 0.0;  // m
  double z = 0.2;  // m
  double roll = 0.0;  // rad
  double pitch = 0.0; // rad
  double yaw = 0.0;   // rad
};

Options parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--group") && i+1 < argc) o.group = argv[++i];
    else if (!strcmp(argv[i], "--x") && i+1 < argc) o.x = std::stod(argv[++i]);
    else if (!strcmp(argv[i], "--y") && i+1 < argc) o.y = std::stod(argv[++i]);
    else if (!strcmp(argv[i], "--z") && i+1 < argc) o.z = std::stod(argv[++i]);
    else if (!strcmp(argv[i], "--roll") && i+1 < argc) o.roll = angles::from_degrees(std::stod(argv[++i]));
    else if (!strcmp(argv[i], "--pitch") && i+1 < argc) o.pitch = angles::from_degrees(std::stod(argv[++i]));
    else if (!strcmp(argv[i], "--yaw") && i+1 < argc) o.yaw = angles::from_degrees(std::stod(argv[++i]));
    else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
      std::cout <<
      "Usage:\n"
      "  ros2 run moveit_joint_stepper_cpp move_pose_target \\\n"
      "    --x 0.2 --y 0.0 --z 0.25 --roll 0 --pitch 90 --yaw 0\n"
      "\nAngles in degrees.\n";
      std::exit(0);
    }
  }
  return o;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_pose_target");

  // Executor 스레드 (joint 코드와 동일)
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_node(node);
  std::atomic<bool> spinning{true};
  std::thread spin_thread([&](){
    while (spinning && rclcpp::ok()) {
      exec->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  auto opt = parse(argc, argv);
  RCLCPP_INFO(node->get_logger(), "Target (x,y,z)=(%.3f, %.3f, %.3f), rpy(deg)=(%.1f,%.1f,%.1f)",
              opt.x, opt.y, opt.z,
              angles::to_degrees(opt.roll),
              angles::to_degrees(opt.pitch),
              angles::to_degrees(opt.yaw));

  moveit::planning_interface::MoveGroupInterface move_group(node, opt.group);
  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);
  move_group.setStartStateToCurrentState();

  // Pose 목표 생성
  tf2::Quaternion q;
  q.setRPY(opt.roll, opt.pitch, opt.yaw);

  geometry_msgs::msg::Pose target;
  target.position.x = opt.x;
  target.position.y = opt.y;
  target.position.z = opt.z;
  target.orientation = tf2::toMsg(q);

  move_group.setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto ok = move_group.plan(plan);

  if (ok != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
    spinning = false; spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto exec_ok = move_group.execute(plan);
  if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
    spinning = false; spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "✅ Move completed.");

  spinning = false;
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
