#include "pb2025_sentry_behavior/plugins/action/manage_sentry_posture.hpp"

#include <cmath>
#include <sstream>
#include <vector>

namespace pb2025_sentry_behavior
{

ManageSentryPostureAction::ManageSentryPostureAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: SyncActionNode(name, config), node_(params.nh)
{
  publisher_ = node_->create_publisher<pb_rm_interfaces::msg::SentryPosture>(
    "/cmd/sentry_posture", 1);
}

uint8_t selectSentryPosture(
  const pb_rm_interfaces::msg::RobotStatus & status, const std::string & vision,
  bool vision_fresh, bool recovery)
{
  uint8_t posture = pb_rm_interfaces::msg::SentryPosture::POSTURE_MOVE;

  if (recovery) {
    posture = status.current_hp < 100 && status.powerful_movement_posture == 1 ?
      pb_rm_interfaces::msg::SentryPosture::POWER_MOVE :
      pb_rm_interfaces::msg::SentryPosture::POSTURE_MOVE;
  } else {
    bool detected = false;

    double distance = 0.0;

    if (vision_fresh && !vision.empty()) {
      std::stringstream stream(vision);

      std::string token;

      std::vector<std::string> parts;

      while (std::getline(stream, token, ',')) parts.push_back(token);

      if (parts.size() >= 3) {
        try {
          const double x = std::stod(parts[0]);

          const double y = std::stod(parts[1]);

          detected = std::stod(parts[2]) == 1.0 && std::isfinite(x) && std::isfinite(y);

          distance = std::hypot(x, y);
        } catch (const std::exception &) {
          detected = false;
        }
      }
    }

    if (detected) {
      posture = distance <= 5.0 && status.powerful_offensive_posture == 1 ?
        pb_rm_interfaces::msg::SentryPosture::POWER_ATTACK :
        pb_rm_interfaces::msg::SentryPosture::POSTURE_ATTACK;
    } else if (status.is_hp_deduced) {
      posture = status.current_hp < 250 && status.powerful_defensive_posture == 1 ?
        pb_rm_interfaces::msg::SentryPosture::POWER_DEFENSE :
        pb_rm_interfaces::msg::SentryPosture::POSTURE_DEFENSE;
    }
  }
  return posture;
}

BT::NodeStatus ManageSentryPostureAction::tick()
{
  auto status = getInput<pb_rm_interfaces::msg::RobotStatus>("robot_status");

  auto vision = getInput<std_msgs::msg::String>("vision_data");

  auto received = getInput<std::chrono::steady_clock::time_point>("vision_received_at");
  
  auto mode = getInput<std::string>("mode");
  if (!status || !mode) return BT::NodeStatus::FAILURE;

  const bool vision_fresh = vision && received &&
    std::chrono::steady_clock::now() - received.value() <= std::chrono::milliseconds(500);
  const auto posture = selectSentryPosture(
    *status, vision ? vision->data : "", vision_fresh, mode.value() == "recovery");

  if (posture != last_posture_) {
    pb_rm_interfaces::msg::SentryPosture message;
    message.current_posture = posture;
    publisher_->publish(message);
    last_posture_ = posture;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList ManageSentryPostureAction::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "robot_status", "{@referee_robotStatus}", "裁判系统机器人状态"),
    BT::InputPort<std_msgs::msg::String>(
      "vision_data", "{@vision_string_data}", "视觉目标数据"),
    BT::InputPort<std::chrono::steady_clock::time_point>(
      "vision_received_at", "{@vision_received_at}", "视觉消息接收时间"),
    BT::InputPort<std::string>("mode", "patrol", "patrol or recovery"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::ManageSentryPostureAction, "ManageSentryPosture");
