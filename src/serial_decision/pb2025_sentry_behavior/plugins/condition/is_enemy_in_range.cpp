#include "pb2025_sentry_behavior/plugins/condition/is_enemy_in_range.hpp"

#include <cmath>

namespace pb2025_sentry_behavior
{

IsEnemyInRangeCondition::IsEnemyInRangeCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsEnemyInRangeCondition::checkEnemyInRange, this), config)
{
}

BT::NodeStatus IsEnemyInRangeCondition::checkEnemyInRange()
{
  auto msg = getInput<std_msgs::msg::String>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "Vision string message is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string current_data = msg.value().data;
  if (current_data.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // vision format: x,y,flag,id
  std::vector<std::string> parts;
  std::stringstream ss(current_data);
  std::string item;
  while (std::getline(ss, item, ',')) {
    parts.push_back(item);
  }

  if (parts.size() < 4) {
    return BT::NodeStatus::FAILURE;
  }

  // 先看有没有检测到敌人
  if (parts[2] != "1" && parts[2] != "1.0" && parts[2] != "1.000000") {
    return BT::NodeStatus::FAILURE;
  }

  // 解析距离
  double max_distance;
  if (!getInput<double>("max_distance", max_distance)) {
    RCLCPP_ERROR(logger_, "Missing required input [max_distance]");
    return BT::NodeStatus::FAILURE;
  }

  double x = std::stod(parts[0]);
  double y = std::stod(parts[1]);
  double distance = std::sqrt(x * x + y * y);

  if (distance <= max_distance) {
    RCLCPP_INFO(logger_, "Enemy in range: dist=%.2f <= max=%.2f (x=%.2f, y=%.2f)",
                distance, max_distance, x, y);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_DEBUG(logger_, "Enemy out of range: dist=%.2f > max=%.2f", distance, max_distance);
  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsEnemyInRangeCondition::providedPorts()
{
  return {
    BT::InputPort<std_msgs::msg::String>(
      "key_port", "{@vision_string_data}", "Vision string data port on blackboard"),
    BT::InputPort<double>(
      "max_distance", 5.0, "Maximum engagement distance (meters)"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsEnemyInRangeCondition>("IsEnemyInRange");
}
