#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal_bb.hpp"

namespace pb2025_sentry_behavior
{

PubNav2GoalBBAction::PubNav2GoalBBAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubNav2GoalBBAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if (!res) {
    RCLCPP_ERROR(node_->get_logger(), "PubNav2GoalBB: 读取端口 [goal_pose] 失败: %s", res.error().c_str());
    return false;
  }

  msg = res.value();

  if (msg.header.frame_id.empty()) {
    auto frame_res = getInput<std::string>("frame_id");
    msg.header.frame_id = (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";
  }

  msg.header.stamp = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "PubNav2GoalBB: 发布目标点 [%.2f, %.2f, %.2f] frame: %s",
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
    msg.header.frame_id.c_str());

  return true;
}

BT::PortsList PubNav2GoalBBAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "导航目标位置"),
    BT::InputPort<std::string>("frame_id", "map", "坐标系ID，默认为map"),
  });
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalBBAction, "PubNav2GoalBB");
