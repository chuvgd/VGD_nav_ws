#include "pb2025_sentry_behavior/plugins/condition/is_last_waypoint.hpp"

namespace pb2025_sentry_behavior
{

IsLastWaypointCondition::IsLastWaypointCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsLastWaypointCondition::checkIsLastWaypoint, this), config)
{
}

BT::NodeStatus IsLastWaypointCondition::checkIsLastWaypoint()
{
  int wp_idx, total;
  if (!getInput<int>("wp_idx", wp_idx) || !getInput<int>("total_waypoints", total)) {
    return BT::NodeStatus::FAILURE;
  }
  return (wp_idx == total - 1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsLastWaypointCondition::providedPorts()
{
  return {
    BT::InputPort<int>("wp_idx", "当前航点索引"),
    BT::InputPort<int>("total_waypoints", "航点总数"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsLastWaypointCondition>("IsLastWaypoint");
}
