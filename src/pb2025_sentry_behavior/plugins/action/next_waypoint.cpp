#include "pb2025_sentry_behavior/plugins/action/next_waypoint.hpp"

#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

NextWaypoint::NextWaypoint(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus NextWaypoint::tick()
{
  auto idx_res = getInput<int>("wp_idx");
  auto total_res = getInput<int>("total_waypoints");

  if (!idx_res || !total_res) {
    RCLCPP_ERROR(rclcpp::get_logger("NextWaypoint"), "读取端口失败");
    return BT::NodeStatus::FAILURE;
  }

  int idx = idx_res.value();
  int total = total_res.value();
  int next = idx + 1;

  // 到达最后一个航点，不再切换，返回 FAILURE 终止外层循环
  if (next >= total) {
    RCLCPP_INFO(
      rclcpp::get_logger("NextWaypoint"), "已到达最后一个航点 (idx=%d, total=%d)，导航结束", idx,
      total);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("NextWaypoint"), "航点切换: %d -> %d (共 %d 个)", idx, next, total);

  setOutput("wp_idx", next);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList NextWaypoint::providedPorts()
{
  return {
    BT::BidirectionalPort<int>("wp_idx", "当前航点索引"),
    BT::InputPort<int>("total_waypoints", "航点总数"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::NextWaypoint>("NextWaypoint");
}
