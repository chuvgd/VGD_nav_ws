#include "pb2025_sentry_behavior/plugins/action/load_waypoints.hpp"

#include "pb2025_sentry_behavior/waypoint_loader.hpp"

namespace pb2025_sentry_behavior
{

LoadWaypoints::LoadWaypoints(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus LoadWaypoints::tick()
{
  auto file_res = getInput<std::string>("waypoint_file");
  if (!file_res) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LoadWaypoints"), "读取端口 [waypoint_file] 时出错: %s",
      file_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto frame_res = getInput<std::string>("frame_id");
  const std::string frame_id =
    (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  std::vector<double> wait_times;
  if (!loadWaypointsFromCSV(file_res.value(), waypoints, frame_id, &wait_times)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LoadWaypoints"), "无法从文件加载航点: %s",
      file_res.value().c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("LoadWaypoints"), "已加载 %lu 个航点",
    static_cast<unsigned long>(waypoints.size()));

  setOutput("waypoints", waypoints);
  setOutput("wait_times", wait_times);
  setOutput("total_waypoints", static_cast<int>(waypoints.size()));
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList LoadWaypoints::providedPorts()
{
  return {
    BT::InputPort<std::string>("waypoint_file", "航点 CSV 文件的绝对路径"),
    BT::InputPort<std::string>("frame_id", "map", "坐标系 ID"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "航点列表"),
    BT::OutputPort<std::vector<double>>("wait_times", "每个航点的等待时间 (秒)"),
    BT::OutputPort<int>("total_waypoints", "航点总数"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::LoadWaypoints>("LoadWaypoints");
}
