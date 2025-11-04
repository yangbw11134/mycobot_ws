#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <angles/angles.h>

#include <string>
#include <vector>
#include <iostream>
#include <cstring>
#include <thread>
#include <atomic>
#include <chrono>

struct Options {
  std::string group = "arm_group"; // 기본 그룹명: 네 환경에 맞게 실행 시 --group 으로 변경 가능
  int joint_index = 2;             // 0-based -> 2 == "3번 관절"
  double delta_deg = 30.0;         // +30도
};

static Options parse_args(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--group") && i + 1 < argc)      opt.group = argv[++i];
    else if (!std::strcmp(argv[i], "--joint") && i + 1 < argc) opt.joint_index = std::stoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--delta") && i + 1 < argc) opt.delta_deg = std::stod(argv[++i]);
    else if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
      std::cout <<
        "Usage: ros2 run moveit_joint_stepper_cpp move_joint_delta "
        "[--group <group_name>] [--joint <0-based-index>] [--delta <degrees>]\n"
        "Defaults: --group arm_group --joint 2 --delta 30\n";
      std::exit(0);
    }
  }
  return opt;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_joint_delta");

  // ✅ Executor를 별도 스레드에서 돌려 콜백/토픽 수신 보장
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_node(node);
  std::atomic<bool> spinning{true};
  std::thread spin_thread([&](){
    while (spinning && rclcpp::ok()) {
      exec->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  const auto opt = parse_args(argc, argv);
  RCLCPP_INFO(node->get_logger(), "group=%s, joint_index=%d, delta_deg=%.3f",
              opt.group.c_str(), opt.joint_index, opt.delta_deg);

  // MoveGroupInterface 준비
  moveit::planning_interface::MoveGroupInterface move_group(node, opt.group);
  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);
  move_group.setStartStateToCurrentState();  // 시작 = 현재

  // 현재 상태 대기 (네트워크/시간 지연 대비 20초)
  auto current_state = move_group.getCurrentState(20.0);
  if (!current_state) {
    RCLCPP_ERROR(node->get_logger(),
      "현재 로봇 상태를 가져오지 못했습니다. bringup/robot_state_publisher/move_group, "
      "네트워크/시간동기(ROS_DOMAIN_ID=%s) 확인!", std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "unset");
    spinning = false; spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup(opt.group);
  if (!jmg) {
    RCLCPP_ERROR(node->get_logger(), "JointModelGroup '%s' 을(를) 찾을 수 없습니다.", opt.group.c_str());
    spinning = false; spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  std::vector<double> joints;
  current_state->copyJointGroupPositions(jmg, joints);

  if (opt.joint_index < 0 || opt.joint_index >= static_cast<int>(joints.size())) {
    RCLCPP_ERROR(node->get_logger(), "joint_index %d 가 범위를 벗어났습니다. (0 ~ %zu)",
                 opt.joint_index, joints.size() - 1);
    spinning = false; spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // (선택) 디버그: 그룹 내 조인트 이름/순서 확인하고 싶을 때 주석 해제
  // {
  //   auto names = jmg->getVariableNames();
  //   for (size_t i = 0; i < names.size(); ++i)
  //     RCLCPP_INFO(node->get_logger(), "joint[%zu]=%s (rad=%.3f)", i, names[i].c_str(), joints[i]);
  // }

  // 델타 적용 (deg -> rad)
  const double delta_rad = angles::from_degrees(opt.delta_deg);
  const double before = joints[opt.joint_index];
  joints[opt.joint_index] += delta_rad;

  // joint limit 충족 검증/보정
  moveit::core::RobotState tmp_state(*current_state);
  tmp_state.setJointGroupPositions(jmg, joints);
  if (!tmp_state.satisfiesBounds(jmg)) {
    RCLCPP_WARN(node->get_logger(), "목표가 조인트 한계를 넘습니다. 한계 내로 보정합니다.");
    tmp_state.enforceBounds(jmg);
    tmp_state.copyJointGroupPositions(jmg, joints);
  }

  // 목표 설정 및 플랜/실행
  move_group.setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = move_group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "플래닝 실패 (error=%d). RViz 충돌/제약/목표 확인!", plan_result.val);
    spinning = false; spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "3번 관절(0-based %d) : %.3f rad -> %.3f rad (delta=%.3f rad)",
              opt.joint_index, before, joints[opt.joint_index], delta_rad);

  auto exec_result = move_group.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "실행 실패 (error=%d).", exec_result.val);
    spinning = false; spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "동작 완료.");

  // 종료
  spinning = false;
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
