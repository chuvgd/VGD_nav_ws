#include "pb2025_sentry_behavior/plugins/condition/is_power_posture_available.hpp"

namespace pb2025_sentry_behavior
{

IsPowerPostureAvailableCondition::IsPowerPostureAvailableCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsPowerPostureAvailableCondition::checkPowerPostureAvailable, this), config)
{
}

BT::NodeStatus IsPowerPostureAvailableCondition::checkPowerPostureAvailable()
{
  // 1. 读取目标强化姿态类型
  uint8_t type;
  if (!getInput<uint8_t>("type", type)) {
    RCLCPP_ERROR(logger_, "Missing required input [type]");
    return BT::NodeStatus::FAILURE;
  }

  constexpr uint8_t POWER_ATTACK = 4;
  constexpr uint8_t POWER_DEFENSE = 5;
  constexpr uint8_t POWER_MOVE = 6;

  if (type != POWER_ATTACK && type != POWER_DEFENSE && type != POWER_MOVE) {
    RCLCPP_ERROR(logger_, "Invalid type: %d. Must be 4 (Attack), 5 (Defense) or 6 (Move)", type);
    return BT::NodeStatus::FAILURE;
  }

  // 2. 读取裁判系统 RobotStatus
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  auto hp_threshold_ = getInput<uint8_t>("hp_threshold");
  uint16_t hp_threshold = hp_threshold_.value();
  if(!hp_threshold){
    hp_threshold = 100;
  }

  bool available = false;

  switch (type) {
    case POWER_ATTACK: {
      // 进攻强化：裁判系统 flag 是否还有剩余时间
      available = (msg->powerful_offensive_posture == 1);
      RCLCPP_DEBUG(logger_, "PowerAttack: flag=%d → %s",
                   msg->powerful_offensive_posture, available ? "AVAILABLE" : "UNAVAILABLE");
      break;
    }

    case POWER_DEFENSE: {
      // 防御强化：flag 还有 + HP 低于阈值才值得用
      bool flag_ok = (msg->powerful_defensive_posture == 1);
      bool hp_low = (msg->current_hp < hp_threshold);
      available = flag_ok && hp_low;
      RCLCPP_DEBUG(logger_, "PowerDefense: flag=%d → %s",
                   msg->powerful_defensive_posture, available ? "AVAILABLE" : "UNAVAILABLE");
      break;
    }

    case POWER_MOVE:{
      // 移动强化：裁判系统 flag 是否还有剩余时间 + HP 阈值
      bool hp_low = (msg->current_hp < hp_threshold);
      bool flag_ok = (msg->powerful_movement_posture == 1);
      available = flag_ok && hp_low;
      RCLCPP_DEBUG(logger_, "PowerMove: flag=%d → %s",
                   msg->powerful_movement_posture, available ? "AVAILABLE" : "UNAVAILABLE");
      break;
    }
  }

  if (available) {
    RCLCPP_INFO(logger_, "PowerPosture type=%d is AVAILABLE", type);
  }

  return available ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsPowerPostureAvailableCondition::providedPorts()
{
  return {
    BT::InputPort<uint8_t>("type", "Reinforcement posture type: 4=PowerAttack, 5=PowerDefense, 6=PowerMove"),
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<uint16_t>(
      "hp_threshold", 100, "HP threshold for PowerDefense (only used when type=5)"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsPowerPostureAvailableCondition>(
    "IsPowerPostureAvailable");
}
