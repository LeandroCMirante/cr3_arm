#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char GROUP_NAME[] = "cr3_arm";
constexpr char TOOL_LINK[] = "gripper_body_link";
constexpr char LIDAR_TOPIC[] = "/box_measurement";
constexpr char ELECTROMAGNET_TOPIC[] = "/garra_command";
constexpr char ATTACH_TOPIC[] = "/sim/electromagnet_attach";

constexpr double PLANNING_TIME_S = 8.0;
constexpr int NUM_PLANNING_ATTEMPTS = 10;
constexpr double MAX_VEL_SCALE = 0.2;
constexpr double MAX_ACC_SCALE = 0.2;

constexpr double LIDAR_Z_REFERENCE_10CM = 0.335;  // m
constexpr double Z_OFFSET_SAFETY_MM = 95.0;       // mm
constexpr double CAMERA_ROTATION_OFFSET_DEG = 30.0;

constexpr double PREGRASP_LIFT_M = 0.06;
constexpr double RETREAT_LIFT_M = 0.08;
constexpr double MIN_BOX_DIM_M = 0.01;

// constants.py
const std::string POINT_MEASURE = "Go to 'Measure' point (above new box)";
const std::string POINT_TAKE_BOX = "Go to 'Take Box' point (grip new box)";
const std::string POINT_CHANGE_ABOVE = "Go to 'Change Area Above' point";
const std::string POINT_CHANGE_SLOT = "Place in 'Change Area Slot' (temporary)";
const std::string POINT_SLOT_1 = "Place in 'Slot 1'";
const std::string POINT_SLOT_2 = "Place in 'Slot 2'";
const std::string POINT_SLOT_3 = "Place in 'Slot 3'";
const std::string POINT_SLOT_4 = "Place in 'Slot 4'";
const std::string POINT_ADJUST1 = "Go to 'Adjust 1' (above slot 1)";
const std::string POINT_ADJUST2 = "Go to 'Adjust 2' (above slot 2)";
const std::string POINT_ADJUST3 = "Go to 'Adjust 3' (above slot 3)";
const std::string POINT_ADJUST4 = "Go to 'Adjust 4' (above slot 4)";

const std::array<std::string, 4> SLOT_POINTS = {
  POINT_SLOT_1, POINT_SLOT_2, POINT_SLOT_3, POINT_SLOT_4};

const std::array<std::string, 4> ADJUST_POINTS = {
  POINT_ADJUST1, POINT_ADJUST2, POINT_ADJUST3, POINT_ADJUST4};

struct CartesianPointMmDeg
{
  double x_mm;
  double y_mm;
  double z_mm;
  double rx_deg;
  double ry_deg;
  double rz_deg;
};

struct BoxMeasurement
{
  double volume_m3{0.0};

  double center_x_m{0.0};
  double center_y_m{0.0};
  double center_z_m{0.0};

  double grasp_x_m{0.0};
  double grasp_y_m{0.0};
  double grasp_z_m{0.0};

  double size_x_m{0.06};
  double size_y_m{0.06};
  double size_z_m{0.10};

  double yaw_rad{0.0};
};

struct SlotState
{
  double volume{0.0};
  double z_offset_mm{0.0};
};

double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

std::vector<double> jointsDegToRad(const std::vector<double> & joints_deg)
{
  std::vector<double> joints_rad;
  joints_rad.reserve(joints_deg.size());
  for (double v : joints_deg) {
    joints_rad.push_back(deg2rad(v));
  }
  return joints_rad;
}

geometry_msgs::msg::Pose cartesianMmDegToPose(const CartesianPointMmDeg & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.x_mm / 1000.0;
  pose.position.y = p.y_mm / 1000.0;
  pose.position.z = p.z_mm / 1000.0;

  tf2::Quaternion q;
  q.setRPY(deg2rad(p.rx_deg), deg2rad(p.ry_deg), deg2rad(p.rz_deg));
  q.normalize();
  pose.orientation = tf2::toMsg(q);

  return pose;
}

BoxMeasurement parseMeasurement(const std_msgs::msg::Float64MultiArray & msg)
{
  BoxMeasurement m{};

  if (msg.data.size() >= 11) {
    m.volume_m3 = msg.data[0];

    m.center_x_m = msg.data[1];
    m.center_y_m = msg.data[2];
    m.center_z_m = msg.data[3];

    m.grasp_x_m = msg.data[4];
    m.grasp_y_m = msg.data[5];
    m.grasp_z_m = msg.data[6];

    m.size_x_m = msg.data[7];
    m.size_y_m = msg.data[8];
    m.size_z_m = msg.data[9];

    m.yaw_rad = msg.data[10];
  } else if (msg.data.size() >= 4) {
    // modo legado: [volume, off_x, off_y, off_z]
    m.volume_m3 = msg.data[0];

    m.center_x_m = msg.data[1];
    m.center_y_m = msg.data[2];
    m.center_z_m = msg.data[3];

    m.grasp_x_m = msg.data[1];
    m.grasp_y_m = msg.data[2];
    m.grasp_z_m = msg.data[3];

    m.size_x_m = 0.06;
    m.size_y_m = 0.06;
    m.size_z_m = 0.10;
    m.yaw_rad = 0.0;
  }

  return m;
}

// Poses do constants.py
const std::vector<double> HOME_JOINTS_DEG = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const std::vector<double> MEASURE_JOINTS_DEG = {
  20.610082, -2.307996, -36.182922, -60.354858, 89.204308, -0.461888};

const std::map<std::string, CartesianPointMmDeg> CARTESIAN_DB = {
  {POINT_CHANGE_ABOVE, {-229.1447, -46.2188, -51.2931, 12.3037, 93.3664, -0.0255}},
  {POINT_CHANGE_SLOT, {-213.5703, 496.9577, -178.7658, 180.0, 0.0, 36.0728}},
  {POINT_ADJUST1, {-61.7499, -38.2041, -33.5323, -14.7636, 89.2174, -0.4578}},
  {POINT_ADJUST2, {-85.0780, -38.2037, -33.5330, -14.7642, 89.2169, -0.4578}},
  {POINT_ADJUST3, {-113.6405, -38.2093, -33.5330, -14.7642, 89.2169, -0.4578}},
  {POINT_ADJUST4, {-144.7971, -38.2085, -33.5326, -14.7640, 89.2169, -0.4578}},
  {POINT_SLOT_1, {179.2547, -531.1418, -178.8560, -180.0, 0.0, -148.6614}},
  {POINT_SLOT_2, {-77.4851, -555.3969, -178.5499, -180.0, 0.0, -175.2552}},
  {POINT_SLOT_3, {-300.5411, -473.4385, -178.5509, -180.0, 0.0, 160.2795}},
  {POINT_SLOT_4, {-510.4340, -232.2213, -158.5502, -180.0, 0.0, 127.1501}},
};

}  // namespace

class Cr3BoxSorterMoveIt : public rclcpp::Node
{
public:
  explicit Cr3BoxSorterMoveIt(const rclcpp::NodeOptions & options)
  : Node("cr3_box_sorter_moveit", options)
  {
    electromagnet_pub_ = create_publisher<std_msgs::msg::String>(ELECTROMAGNET_TOPIC, 10);
    attach_pub_ = create_publisher<std_msgs::msg::Bool>(ATTACH_TOPIC, 10);

    lidar_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      LIDAR_TOPIC, 10,
      std::bind(&Cr3BoxSorterMoveIt::onLidarMeasurement, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Node created. Waiting for explicit MoveIt initialization...");
  }

  void init()
  {
    using moveit::planning_interface::MoveGroupInterface;

    move_group_ = std::make_unique<MoveGroupInterface>(shared_from_this(), GROUP_NAME);
    move_group_->setPlanningTime(PLANNING_TIME_S);
    move_group_->setNumPlanningAttempts(NUM_PLANNING_ATTEMPTS);
    move_group_->setMaxVelocityScalingFactor(MAX_VEL_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACC_SCALE);
    move_group_->setEndEffectorLink(TOOL_LINK);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.05);

    planning_frame_ = move_group_->getPlanningFrame();
    tool_link_ = move_group_->getEndEffectorLink();

    timer_ = create_wall_timer(1500ms, std::bind(&Cr3BoxSorterMoveIt::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "CR3 box sorter ready. Planning frame: %s | tool link: %s",
      planning_frame_.c_str(), tool_link_.c_str());
    RCLCPP_INFO(get_logger(), "Waiting for LiDAR measurements on %s", LIDAR_TOPIC);
  }

private:
  enum class State
  {
    STARTUP_HOME,
    GO_MEASURE,
    WAIT_LIDAR,
    PROCESS_BOX
  };

  void onLidarMeasurement(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.empty()) {
      return;
    }

    pending_measurement_ = parseMeasurement(*msg);
    lidar_ready_ = true;

    RCLCPP_INFO(
      get_logger(),
      "LiDAR measurement received: volume=%.6f center=(%.3f, %.3f, %.3f) grasp=(%.3f, %.3f, %.3f)",
      pending_measurement_.volume_m3,
      pending_measurement_.center_x_m, pending_measurement_.center_y_m, pending_measurement_.center_z_m,
      pending_measurement_.grasp_x_m, pending_measurement_.grasp_y_m, pending_measurement_.grasp_z_m);
  }

  void tick()
  {
    if (!move_group_) {
      return;
    }

    switch (state_) {
      case State::STARTUP_HOME:
        if (goToJointTargetDeg(HOME_JOINTS_DEG, "home")) {
          state_ = State::GO_MEASURE;
        }
        break;

      case State::GO_MEASURE:
        if (goToJointTargetDeg(MEASURE_JOINTS_DEG, "measure")) {
          state_ = State::WAIT_LIDAR;
          RCLCPP_INFO(get_logger(), "Robot at measure pose. Waiting for box measurement...");
        }
        break;

      case State::WAIT_LIDAR:
        if (lidar_ready_) {
          state_ = State::PROCESS_BOX;
        }
        break;

      case State::PROCESS_BOX:
        processPendingBox();
        state_ = State::GO_MEASURE;
        break;
    }
  }

  bool processPendingBox()
  {
    if (!lidar_ready_) {
      return false;
    }

    lidar_ready_ = false;
    const auto box = pending_measurement_;

    // Mantém a mesma lógica antiga de empilhamento por delta Z do LiDAR
    const double delta_z_mm = (LIDAR_Z_REFERENCE_10CM - box.center_z_m) * 1000.0;

    int insert_index = -1;
    for (int i = 0; i < 4; ++i) {
      if (slots_[i].volume <= 0.0) {
        insert_index = i;
        break;
      }
      if (box.volume_m3 < slots_[i].volume) {
        insert_index = i;
        break;
      }
    }

    if (insert_index < 0) {
      RCLCPP_WARN(get_logger(), "No free slot found. Sending box to change area.");
      return pickAndPlaceToNamedPoint(
        box, POINT_CHANGE_ABOVE, POINT_CHANGE_SLOT, delta_z_mm, "change_area_box");
    }

    const bool requires_shuffle = (slots_[insert_index].volume > 0.0);
    if (requires_shuffle) {
      RCLCPP_WARN(
        get_logger(),
        "Sorted insertion at slot %d would require shuffle. Current version uses change area as fallback.",
        insert_index + 1);
      return pickAndPlaceToNamedPoint(
        box, POINT_CHANGE_ABOVE, POINT_CHANGE_SLOT, delta_z_mm, "change_area_box");
    }

    const bool ok = pickAndPlaceToNamedPoint(
      box,
      ADJUST_POINTS[insert_index],
      SLOT_POINTS[insert_index],
      delta_z_mm,
      "slot_box_" + std::to_string(insert_index + 1));

    if (ok) {
      slots_[insert_index].volume = box.volume_m3;
      slots_[insert_index].z_offset_mm = delta_z_mm;
      logSlots();
    }

    return ok;
  }

  bool pickAndPlaceToNamedPoint(
    const BoxMeasurement & box,
    const std::string & adjust_point_name,
    const std::string & place_point_name,
    double z_offset_mm,
    const std::string & object_id)
  {
    addDetectedBoxCollision(box, object_id);

    // 1) pre-grasp
    auto pre_grasp = measuredGripPose(box);
    pre_grasp.position.z += PREGRASP_LIFT_M;

    if (!planAndExecutePose(pre_grasp, "pre_grasp")) {
      removeCollisionObject(object_id);
      return false;
    }

    // 2) descend
    auto grasp_pose = measuredGripPose(box);
    if (!computeAndExecuteCartesian({grasp_pose}, "descend_to_grasp")) {
      removeCollisionObject(object_id);
      return false;
    }

    // 3) attach / energize
    setElectromagnet(true);
    attachCollisionObject(object_id);

    // 4) retreat
    auto retreat_pose = grasp_pose;
    retreat_pose.position.z += RETREAT_LIFT_M;
    if (!computeAndExecuteCartesian({retreat_pose}, "retreat_after_grasp")) {
      detachCollisionObject(object_id);
      removeCollisionObject(object_id);
      setElectromagnet(false);
      return false;
    }

    // 5) move to adjust point
    const auto adjust_it = CARTESIAN_DB.find(adjust_point_name);
    if (adjust_it == CARTESIAN_DB.end()) {
      RCLCPP_ERROR(get_logger(), "Adjust point not found: %s", adjust_point_name.c_str());
      detachCollisionObject(object_id);
      removeCollisionObject(object_id);
      setElectromagnet(false);
      return false;
    }

    if (!planAndExecutePose(cartesianMmDegToPose(adjust_it->second), adjust_point_name)) {
      detachCollisionObject(object_id);
      removeCollisionObject(object_id);
      setElectromagnet(false);
      return false;
    }

    // 6) place point + z offset
    const auto slot_it = CARTESIAN_DB.find(place_point_name);
    if (slot_it == CARTESIAN_DB.end()) {
      RCLCPP_ERROR(get_logger(), "Place point not found: %s", place_point_name.c_str());
      detachCollisionObject(object_id);
      removeCollisionObject(object_id);
      setElectromagnet(false);
      return false;
    }

    CartesianPointMmDeg place_point = slot_it->second;
    place_point.z_mm += z_offset_mm;
    const auto place_pose = cartesianMmDegToPose(place_point);

    if (!planAndExecutePose(place_pose, place_point_name)) {
      detachCollisionObject(object_id);
      removeCollisionObject(object_id);
      setElectromagnet(false);
      return false;
    }

    // 7) release
    detachCollisionObject(object_id);
    setElectromagnet(false);
    removeCollisionObject(object_id);

    // 8) retreat to adjust again
    if (!planAndExecutePose(cartesianMmDegToPose(adjust_it->second), adjust_point_name + "_retreat")) {
      return false;
    }

    return true;
  }

  geometry_msgs::msg::Pose measuredGripPose(const BoxMeasurement & box) const
  {
    geometry_msgs::msg::Pose pose;

    // Por enquanto assumimos que a medição já vem no frame de planejamento.
    // Futuramente: transformar de lidar_link -> planning_frame via TF.
    pose.position.x = box.grasp_x_m;
    pose.position.y = box.grasp_y_m;
    pose.position.z = box.grasp_z_m + (Z_OFFSET_SAFETY_MM / 1000.0);

    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, box.yaw_rad + deg2rad(CAMERA_ROTATION_OFFSET_DEG));
    q.normalize();
    pose.orientation = tf2::toMsg(q);

    return pose;
  }

  bool goToJointTargetDeg(const std::vector<double> & joints_deg, const std::string & label)
  {
    move_group_->setJointValueTarget(jointsDegToRad(joints_deg));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = static_cast<bool>(move_group_->plan(plan));
    if (!success) {
      RCLCPP_ERROR(get_logger(), "Planning to named joint pose '%s' failed", label.c_str());
      return false;
    }

    const bool exec_ok = static_cast<bool>(move_group_->execute(plan));
    if (!exec_ok) {
      RCLCPP_ERROR(get_logger(), "Execution of named joint pose '%s' failed", label.c_str());
      return false;
    }

    return true;
  }

  bool planAndExecutePose(const geometry_msgs::msg::Pose & target_pose, const std::string & label)
  {
    move_group_->setPoseTarget(target_pose, TOOL_LINK);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = static_cast<bool>(move_group_->plan(plan));
    if (!success) {
      RCLCPP_ERROR(get_logger(), "Pose planning failed for '%s'", label.c_str());
      move_group_->clearPoseTargets();
      return false;
    }

    const bool exec_ok = static_cast<bool>(move_group_->execute(plan));
    move_group_->clearPoseTargets();

    if (!exec_ok) {
      RCLCPP_ERROR(get_logger(), "Pose execution failed for '%s'", label.c_str());
      return false;
    }

    return true;
  }

  bool computeAndExecuteCartesian(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const std::string & label)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;

    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    const double fraction =
      move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.95) {
      RCLCPP_ERROR(
        get_logger(),
        "Cartesian path '%s' incomplete. Fraction=%.3f",
        label.c_str(), fraction);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    const bool exec_ok = static_cast<bool>(move_group_->execute(plan));
    if (!exec_ok) {
      RCLCPP_ERROR(get_logger(), "Cartesian execution failed for '%s'", label.c_str());
      return false;
    }

    return true;
  }

  void addDetectedBoxCollision(const BoxMeasurement & box, const std::string & object_id)
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = planning_frame_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {
      std::max(MIN_BOX_DIM_M, box.size_x_m),
      std::max(MIN_BOX_DIM_M, box.size_y_m),
      std::max(MIN_BOX_DIM_M, box.size_z_m)
    };

    geometry_msgs::msg::Pose pose;
    pose.position.x = box.center_x_m;
    pose.position.y = box.center_y_m;
    pose.position.z = box.center_z_m;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, box.yaw_rad);
    q.normalize();
    pose.orientation = tf2::toMsg(q);

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObject(object);
    current_object_id_ = object_id;
  }

  void removeCollisionObject(const std::string & object_id)
  {
    planning_scene_interface_.removeCollisionObjects({object_id});
    if (current_object_id_ == object_id) {
      current_object_id_.clear();
    }
  }

  void attachCollisionObject(const std::string & object_id)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = TOOL_LINK;
    aco.object.id = object_id;
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    aco.touch_links = {TOOL_LINK, "Link6", "left_finger_link", "right_finger_link"};
    planning_scene_interface_.applyAttachedCollisionObject(aco);

    publishAttach(true);
  }

  void detachCollisionObject(const std::string & object_id)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = TOOL_LINK;
    aco.object.id = object_id;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_.applyAttachedCollisionObject(aco);

    publishAttach(false);
  }

  void setElectromagnet(bool on)
  {
    std_msgs::msg::String msg;
    msg.data = on ? "ligar" : "desligar";
    electromagnet_pub_->publish(msg);
  }

  void publishAttach(bool on)
  {
    std_msgs::msg::Bool msg;
    msg.data = on;
    attach_pub_->publish(msg);
  }

  void logSlots() const
  {
    std::ostringstream oss;
    oss << "Slots: ";
    for (size_t i = 0; i < slots_.size(); ++i) {
      oss << "[" << i + 1
          << ": vol=" << slots_[i].volume
          << ", z_offset_mm=" << slots_[i].z_offset_mm
          << "] ";
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lidar_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr electromagnet_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr attach_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string planning_frame_;
  std::string tool_link_;
  std::string current_object_id_;

  State state_{State::STARTUP_HOME};
  bool lidar_ready_{false};
  BoxMeasurement pending_measurement_{};

  std::array<SlotState, 4> slots_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Cr3BoxSorterMoveIt>(rclcpp::NodeOptions{});
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}