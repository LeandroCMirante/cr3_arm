#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{
constexpr char GROUP_NAME[] = "cr3_arm";
constexpr char TOOL_LINK[] = "gripper_body_link";
constexpr char ELECTROMAGNET_TOPIC[] = "/garra_command";
constexpr char LIDAR_TOPIC[] = "/box_measurement";
constexpr char DEFAULT_BOX_ID[] = "active_box";
constexpr double APPROACH_CLEARANCE_M = 0.10;  // 10 cm above box
constexpr double PLACE_CLEARANCE_M = 0.10;     // 10 cm above slot
constexpr double POSE_GOAL_TOLERANCE_M = 0.005;
constexpr double ORIENTATION_TOLERANCE_RAD = 0.05;
constexpr double MAX_VEL_SCALE = 0.20;
constexpr double MAX_ACC_SCALE = 0.20;
constexpr double PLANNING_TIME_S = 8.0;
constexpr int NUM_PLANNING_ATTEMPTS = 10;

struct JointPose
{
  std::array<double, 6> joints;
};

struct CartesianPoseMmRpy
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
  std::string frame_id;
  double volume_m3 {0.0};
  geometry_msgs::msg::Pose center_pose;
  geometry_msgs::msg::Pose grasp_pose;
  double size_x_m {0.0};
  double size_y_m {0.0};
  double size_z_m {0.0};
  double yaw_rad {0.0};
  bool valid {false};
};

geometry_msgs::msg::Quaternion quaternionFromRpyDeg(double roll_deg, double pitch_deg, double yaw_deg)
{
  tf2::Quaternion q;
  q.setRPY(roll_deg * M_PI / 180.0, pitch_deg * M_PI / 180.0, yaw_deg * M_PI / 180.0);
  return tf2::toMsg(q);
}

geometry_msgs::msg::Pose poseFromMmRpy(const CartesianPoseMmRpy & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.x_mm / 1000.0;
  pose.position.y = p.y_mm / 1000.0;
  pose.position.z = p.z_mm / 1000.0;
  pose.orientation = quaternionFromRpyDeg(p.rx_deg, p.ry_deg, p.rz_deg);
  return pose;
}

// Poses ported from constants.py provided by the user.
const std::unordered_map<std::string, JointPose> kJointPoses = {
  {"home", {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},
  {"measure", {{20.610082, -2.307996, -36.182922, -60.354858, 89.204308, -0.461888}}},
  {"adjust_1", {{-61.7499, -38.2041, -33.5323, -14.7636, 89.2174, -0.4578}}},
  {"adjust_2", {{-85.078, -38.2037, -33.533, -14.7642, 89.2169, -0.4578}}},
  {"adjust_3", {{-113.6405, -38.2093, -33.533, -14.7642, 89.2169, -0.4578}}},
  {"adjust_4", {{-144.7971, -38.2085, -33.5326, -14.764, 89.2169, -0.4578}}},
  {"change_above", {{-229.1447, -46.2188, -51.2931, 12.3037, 93.3664, -0.0255}}},
  {"adjust_common", {{-110.9599, -42.4403, -53.4256, 29.0344, 93.3654, -0.0255}}},
};

const std::unordered_map<std::string, CartesianPoseMmRpy> kCartesianPoses = {
  {"slot_1", {179.2547, -531.1418, -178.8560, -180.0, 0.0, -148.6614}},
  {"slot_2", {-77.4851, -555.3969, -178.5499, -180.0, 0.0, -175.2552}},
  {"slot_3", {-300.5411, -473.4385, -178.5509, -180.0, 0.0, 160.2795}},
  {"slot_4", {-510.4340, -232.2213, -158.5502, -180.0, 0.0, 127.1501}},
  {"change_slot", {-213.5703, 496.9577, -178.7658, 180.0, 0.0, 36.0728}},
};

const std::array<std::string, 4> kAdjustPoses = {"adjust_1", "adjust_2", "adjust_3", "adjust_4"};
const std::array<std::string, 4> kSlotPoses = {"slot_1", "slot_2", "slot_3", "slot_4"};

class Cr3BoxSorterMoveIt : public rclcpp::Node
{
public:
  explicit Cr3BoxSorterMoveIt(const rclcpp::NodeOptions & options)
  : Node("cr3_box_sorter_moveit", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    using moveit::planning_interface::MoveGroupInterface;

    move_group_ = std::make_unique<MoveGroupInterface>(shared_from_this(), GROUP_NAME);
    move_group_->setPlanningTime(PLANNING_TIME_S);
    move_group_->setNumPlanningAttempts(NUM_PLANNING_ATTEMPTS);
    move_group_->setMaxVelocityScalingFactor(MAX_VEL_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACC_SCALE);
    move_group_->setEndEffectorLink(TOOL_LINK);
    move_group_->setGoalPositionTolerance(POSE_GOAL_TOLERANCE_M);
    move_group_->setGoalOrientationTolerance(ORIENTATION_TOLERANCE_RAD);

    planning_frame_ = move_group_->getPlanningFrame();
    tool_link_ = move_group_->getEndEffectorLink();

    electromagnet_pub_ = create_publisher<std_msgs::msg::String>(ELECTROMAGNET_TOPIC, 10);
    attach_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/electromagnet_attach", 10);
    lidar_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      LIDAR_TOPIC, 10,
      std::bind(&Cr3BoxSorterMoveIt::onLidarMeasurement, this, std::placeholders::_1));

    timer_ = create_wall_timer(1500ms, std::bind(&Cr3BoxSorterMoveIt::tick, this));

    RCLCPP_INFO(get_logger(), "CR3 box sorter ready. Planning frame: %s | tool link: %s",
      planning_frame_.c_str(), tool_link_.c_str());
    RCLCPP_INFO(get_logger(), "Waiting for LiDAR measurements on %s", LIDAR_TOPIC);
  }

private:
  enum class State
  {
    STARTUP,
    MOVE_HOME,
    MOVE_MEASURE,
    WAIT_MEASUREMENT,
    PICK_SEQUENCE,
    PLACE_SEQUENCE,
    IDLE,
  };

  struct SlotState
  {
    double occupied_volume_m3 {0.0};
    double stacked_height_m {0.0};
  };

  void tick()
  {
    try {
      switch (state_) {
        case State::STARTUP:
          if (goToNamedJointPose("home")) {
            state_ = State::MOVE_MEASURE;
          }
          break;

        case State::MOVE_HOME:
          if (goToNamedJointPose("home")) {
            state_ = State::MOVE_MEASURE;
          }
          break;

        case State::MOVE_MEASURE:
          if (goToNamedJointPose("measure")) {
            state_ = State::WAIT_MEASUREMENT;
            RCLCPP_INFO(get_logger(), "At measure pose. Waiting for box measurement...");
          }
          break;

        case State::WAIT_MEASUREMENT:
          if (pending_measurement_.has_value() && pending_measurement_->valid) {
            active_measurement_ = pending_measurement_;
            pending_measurement_.reset();
            updateTargetSlot();
            state_ = State::PICK_SEQUENCE;
          }
          break;

        case State::PICK_SEQUENCE:
          if (executePickSequence(*active_measurement_)) {
            state_ = State::PLACE_SEQUENCE;
          } else {
            RCLCPP_WARN(get_logger(), "Pick sequence failed. Returning to measure pose.");
            state_ = State::MOVE_MEASURE;
          }
          break;

        case State::PLACE_SEQUENCE:
          if (executePlaceSequence(*active_measurement_)) {
            finalizeBoxPlacement(*active_measurement_);
            active_measurement_.reset();
            state_ = State::MOVE_MEASURE;
          } else {
            RCLCPP_WARN(get_logger(), "Place sequence failed. Keeping box attached in scene and returning home.");
            state_ = State::MOVE_HOME;
          }
          break;

        case State::IDLE:
          break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Unhandled exception in state machine: %s", ex.what());
      state_ = State::MOVE_HOME;
    }
  }

  void onLidarMeasurement(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Expected layout for future LiDAR integration:
    // [0] volume_m3
    // [1] center_x, [2] center_y, [3] center_z
    // [4] grasp_x,  [5] grasp_y,  [6] grasp_z
    // [7] size_x,   [8] size_y,   [9] size_z
    // [10] yaw_rad
    // Optional string frame_id can be promoted later to a custom message; for now planning_frame_ is assumed.
    if (msg->data.size() < 11) {
      RCLCPP_WARN(get_logger(), "Ignoring LiDAR message with %zu fields; expected at least 11", msg->data.size());
      return;
    }

    BoxMeasurement m;
    m.frame_id = planning_frame_;
    m.volume_m3 = msg->data[0];
    m.center_pose.position.x = msg->data[1];
    m.center_pose.position.y = msg->data[2];
    m.center_pose.position.z = msg->data[3];
    m.grasp_pose.position.x = msg->data[4];
    m.grasp_pose.position.y = msg->data[5];
    m.grasp_pose.position.z = msg->data[6];
    m.size_x_m = msg->data[7];
    m.size_y_m = msg->data[8];
    m.size_z_m = msg->data[9];
    m.yaw_rad = msg->data[10];

    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, m.yaw_rad);
    m.grasp_pose.orientation = tf2::toMsg(q);
    m.center_pose.orientation = tf2::toMsg(q);
    m.valid = true;

    pending_measurement_ = m;
    RCLCPP_INFO(get_logger(),
      "Received box measurement: volume=%.5f m^3 | center=(%.3f, %.3f, %.3f) | size=(%.3f, %.3f, %.3f)",
      m.volume_m3,
      m.center_pose.position.x, m.center_pose.position.y, m.center_pose.position.z,
      m.size_x_m, m.size_y_m, m.size_z_m);
  }

  bool goToNamedJointPose(const std::string & name)
  {
    auto it = kJointPoses.find(name);
    if (it == kJointPoses.end()) {
      RCLCPP_ERROR(get_logger(), "Unknown named joint pose: %s", name.c_str());
      return false;
    }

    const auto & joints = it->second.joints;
    std::vector<double> target(joints.begin(), joints.end());
    move_group_->setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = static_cast<bool>(move_group_->plan(plan));
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Planning to named joint pose '%s' failed", name.c_str());
      return false;
    }

    const auto result = move_group_->execute(plan);
    return static_cast<bool>(result);
  }

  bool goToPoseTarget(const geometry_msgs::msg::Pose & pose, const std::string & frame_id)
  {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = frame_id;
    stamped.header.stamp = now();
    stamped.pose = pose;

    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPoseToPlanningFrame(stamped, transformed)) {
      return false;
    }

    move_group_->setPoseTarget(transformed.pose, tool_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = static_cast<bool>(move_group_->plan(plan));
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Pose planning failed");
      move_group_->clearPoseTargets();
      return false;
    }

    const auto result = move_group_->execute(plan);
    move_group_->clearPoseTargets();
    return static_cast<bool>(result);
  }

  bool goToPoseRelativeToCurrent(double dx_m, double dy_m, double dz_m)
  {
    auto current = move_group_->getCurrentPose(tool_link_).pose;
    current.position.x += dx_m;
    current.position.y += dy_m;
    current.position.z += dz_m;
    return goToPoseTarget(current, planning_frame_);
  }

  bool executePickSequence(const BoxMeasurement & m)
  {
    // Future LiDAR-on-tool integration: if the LiDAR publishes the grasp pose in lidar_link,
    // just set m.frame_id = "lidar_link" and keep the rest of the code unchanged.
    auto approach = m.grasp_pose;
    approach.position.z += APPROACH_CLEARANCE_M;

    if (!goToPoseTarget(approach, m.frame_id)) {
      return false;
    }

    if (!goToPoseTarget(m.grasp_pose, m.frame_id)) {
      return false;
    }

    electromagnetOn();
    attachBoxInScene(m);

    // Retreat vertically.
    if (!goToPoseTarget(approach, m.frame_id)) {
      return false;
    }

    return true;
  }

  bool executePlaceSequence(const BoxMeasurement & m)
  {
    const auto & slot_name = kSlotPoses.at(target_slot_index_);
    const auto & slot_pose_def = kCartesianPoses.at(slot_name);

    geometry_msgs::msg::Pose place_pose = poseFromMmRpy(slot_pose_def);
    place_pose.position.z += slot_states_[target_slot_index_].stacked_height_m;

    geometry_msgs::msg::Pose approach_pose = place_pose;
    approach_pose.position.z += PLACE_CLEARANCE_M;

    if (!goToNamedJointPose(kAdjustPoses.at(target_slot_index_))) {
      return false;
    }

    if (!goToPoseTarget(approach_pose, planning_frame_)) {
      return false;
    }

    if (!goToPoseTarget(place_pose, planning_frame_)) {
      return false;
    }

    detachBoxInScene();
    electromagnetOff();

    if (!goToPoseTarget(approach_pose, planning_frame_)) {
      return false;
    }

    return true;
  }

  void updateTargetSlot()
  {
    // Sort by ascending volume, first empty compatible slot wins.
    int insert_index = -1;
    for (size_t i = 0; i < slot_states_.size(); ++i) {
      if (slot_states_[i].occupied_volume_m3 == 0.0) {
        insert_index = static_cast<int>(i);
        break;
      }
      if (active_measurement_->volume_m3 < slot_states_[i].occupied_volume_m3) {
        insert_index = static_cast<int>(i);
        break;
      }
    }

    if (insert_index < 0) {
      insert_index = 3;  // fallback: last slot
    }

    target_slot_index_ = static_cast<size_t>(insert_index);
    RCLCPP_INFO(get_logger(), "Target slot selected: %zu", target_slot_index_ + 1);
  }

  void finalizeBoxPlacement(const BoxMeasurement & m)
  {
    slot_states_[target_slot_index_].occupied_volume_m3 = m.volume_m3;
    slot_states_[target_slot_index_].stacked_height_m += m.size_z_m;
    RCLCPP_INFO(get_logger(), "Updated slot %zu height to %.3f m",
      target_slot_index_ + 1, slot_states_[target_slot_index_].stacked_height_m);
  }

  void electromagnetOn()
  {
    std_msgs::msg::String msg;
    msg.data = "abrir";  // kept compatible with current stack; adjust if your real electromagnet uses another command.
    electromagnet_pub_->publish(msg);

    std_msgs::msg::Bool attach_msg;
    attach_msg.data = true;
    attach_pub_->publish(attach_msg);
  }

  void electromagnetOff()
  {
    std_msgs::msg::String msg;
    msg.data = "fechar";
    electromagnet_pub_->publish(msg);

    std_msgs::msg::Bool attach_msg;
    attach_msg.data = false;
    attach_pub_->publish(attach_msg);
  }

  void attachBoxInScene(const BoxMeasurement & m)
  {
    // Adds the detected box to the MoveIt planning scene and attaches it to the tool.
    moveit_msgs::msg::CollisionObject object;
    object.id = DEFAULT_BOX_ID;
    object.header.frame_id = planning_frame_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {m.size_x_m, m.size_y_m, m.size_z_m};

    geometry_msgs::msg::PoseStamped grasp_stamped;
    grasp_stamped.header.frame_id = m.frame_id;
    grasp_stamped.header.stamp = now();
    grasp_stamped.pose = m.center_pose;

    geometry_msgs::msg::PoseStamped transformed_center;
    if (!transformPoseToPlanningFrame(grasp_stamped, transformed_center)) {
      return;
    }

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(transformed_center.pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObject(object);
    move_group_->attachObject(DEFAULT_BOX_ID, tool_link_);
  }

  void detachBoxInScene()
  {
    move_group_->detachObject(DEFAULT_BOX_ID);
    planning_scene_interface_.removeCollisionObjects({DEFAULT_BOX_ID});
  }

  bool transformPoseToPlanningFrame(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output)
  {
    if (input.header.frame_id == planning_frame_) {
      output = input;
      return true;
    }

    try {
      output = tf_buffer_.transform(input, planning_frame_, tf2::durationFromSec(0.2));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "TF transform failed from %s to %s: %s",
        input.header.frame_id.c_str(), planning_frame_.c_str(), ex.what());
      return false;
    }
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr electromagnet_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr attach_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string planning_frame_;
  std::string tool_link_;
  std::optional<BoxMeasurement> pending_measurement_;
  std::optional<BoxMeasurement> active_measurement_;
  std::array<SlotState, 4> slot_states_ {};
  size_t target_slot_index_ {0};
  State state_ {State::STARTUP};
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Cr3BoxSorterMoveIt>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
