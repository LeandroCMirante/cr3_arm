#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "cr3_first_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::Logger logger = rclcpp::get_logger("cr3_first_program");
  using moveit::planning_interface::MoveGroupInterface;

  MoveGroupInterface move_group(node, "cr3_arm");

  move_group.setPlanningTime(8.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.setEndEffectorLink("gripper_body_link");

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // =========================================================
  // 1) Meta em juntas
  // =========================================================
  std::vector<double> joint_goal = {
    0.0,    // joint1
    -0.4,   // joint2
    0.8,    // joint3
    0.0,    // joint4
    0.4,    // joint5
    0.0     // joint6
  };

  move_group.setJointValueTarget(joint_goal);

  MoveGroupInterface::Plan joint_plan;
  bool success = static_cast<bool>(move_group.plan(joint_plan));

  if (success) {
    RCLCPP_INFO(logger, "Joint-space plan succeeded. Executing...");
    move_group.execute(joint_plan);
  } else {
    RCLCPP_ERROR(logger, "Joint-space plan failed.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::sleep_for(std::chrono::seconds(2));

  // =========================================================
  // 2) Meta em pose
  // Usa a pose atual como base e desloca pouco
  // =========================================================
  geometry_msgs::msg::Pose target_pose =
    move_group.getCurrentPose("gripper_body_link").pose;

  RCLCPP_INFO(
    logger,
    "Current pose: x=%.3f y=%.3f z=%.3f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  // Pequeno deslocamento para teste
  target_pose.position.x += 0.03;
  target_pose.position.z -= 0.02;

  move_group.setPoseTarget(target_pose, "gripper_body_link");

  MoveGroupInterface::Plan pose_plan;
  success = static_cast<bool>(move_group.plan(pose_plan));

  if (success) {
    RCLCPP_INFO(logger, "Pose plan succeeded. Executing...");
    move_group.execute(pose_plan);
  } else {
    RCLCPP_ERROR(logger, "Pose plan failed.");
  }

  move_group.clearPoseTargets();

  rclcpp::shutdown();
  return 0;
}