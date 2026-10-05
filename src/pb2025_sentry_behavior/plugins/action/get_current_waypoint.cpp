#include "pb2025_sentry_behavior/plugins/action/get_current_waypoint.hpp"

#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

GetCurrentWaypoint::GetCurrentWaypoint(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetCurrentWaypoint::tick()
{
  auto wp_res = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints");
  auto wt_res = getInput<std::vector<double>>("wait_times");
  auto idx_res = getInput<int>("wp_idx");

  if (!wp_res || !idx_res) {
    RCLCPP_ERROR(rclcpp::get_logger("GetCurrentWaypoint"), "读取端口失败");
    return BT::NodeStatus::FAILURE;
  }

  const auto & waypoints = wp_res.value();
  int idx = idx_res.value();

  if (waypoints.empty() || idx < 0 || idx >= static_cast<int>(waypoints.size())) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetCurrentWaypoint"), "航点索引越界: idx=%d, size=%lu", idx,
      static_cast<unsigned long>(waypoints.size()));
    return BT::NodeStatus::FAILURE;
  }

  auto goal = waypoints[idx];
  goal.header.stamp = rclcpp::Clock().now();

  double wait_sec = 0.0;
  if (wt_res && idx < static_cast<int>(wt_res.value().size())) {
    wait_sec = wt_res.value()[idx];
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GetCurrentWaypoint"), "当前目标航点[%d]: [%.2f, %.2f], 等待 %.1fs", idx,
    goal.pose.position.x, goal.pose.position.y, wait_sec);

  setOutput("current_goal", goal);
  setOutput("current_wait_sec", wait_sec);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetCurrentWaypoint::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "航点列表"),
    BT::InputPort<std::vector<double>>("wait_times", "每个航点的等待时间列表"),
    BT::InputPort<int>("wp_idx", "当前航点索引"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_goal", "当前目标点"),
    BT::OutputPort<double>("current_wait_sec", "当前航点的等待时间"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::GetCurrentWaypoint>("GetCurrentWaypoint");
}
