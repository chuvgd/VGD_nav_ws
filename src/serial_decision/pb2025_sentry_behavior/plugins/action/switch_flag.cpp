#include "pb2025_sentry_behavior/plugins/action/switch_flag.hpp"

#include "pb_rm_interfaces/msg/game_robot_hp.hpp"

namespace pb2025_sentry_behavior {

SwitchFlagAction::SwitchFlagAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
  : BT::RosTopicPubNode<pb_rm_interfaces::msg::SwitchFlag>(name, config, params)
{
}

bool SwitchFlagAction::setMessage(pb_rm_interfaces::msg::SwitchFlag & msg)
{
  int wp_idx;
  int total_waypoints;

  if (!getInput<int>("wp_idx", wp_idx)) {
    RCLCPP_ERROR(logger(), "Missing required input [wp_idx]");
    return false;
  }

  if (!getInput<int>("total_waypoints", total_waypoints)) {
    RCLCPP_ERROR(logger(), "Missing required input [total_waypoints]");
    return false;
  }

  // 前哨站存活 + 最后一个路点 → 前哨旋转模式, 否则锁敌旋转模式
  bool outpost_alive = false;
  auto hp_msg = getInput<pb_rm_interfaces::msg::GameRobotHP>("key_port");
  if (hp_msg) {
    outpost_alive = (hp_msg->enemy_outpost_flag == 1);
  }

  if (wp_idx == total_waypoints - 1 && outpost_alive) {
    msg.switch_flag = 1;
  } else {
    msg.switch_flag = 0;
  }

  std::string flag_name = (msg.switch_flag == 1) ? "OutPost" : "Enemy";
  RCLCPP_INFO_STREAM(logger(), "Published Switch Flag Command: [" << flag_name
    << "] (ID: " << static_cast<int>(msg.switch_flag)
    << ", wp_idx: " << wp_idx << "/" << total_waypoints << ")");

  return true;
}

BT::PortsList SwitchFlagAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<int>("wp_idx", "当前航点索引"),
    BT::InputPort<int>("total_waypoints", "航点总数"),
    BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "GameRobotHP port on blackboard"),
  };
  return providedBasicPorts(additional_ports);
}

} // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SwitchFlagAction, "SwitchFlag");