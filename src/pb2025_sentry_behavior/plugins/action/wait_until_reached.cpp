#include "pb2025_sentry_behavior/plugins/action/wait_until_reached.hpp"

#include <cmath>

namespace pb2025_sentry_behavior
{

WaitUntilReached::WaitUntilReached(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config), tolerance_(0.5)
{
  // 创建内部 ROS 节点用于 TF2 查询
  node_ = rclcpp::Node::make_shared("wait_until_reached_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);
}

BT::NodeStatus WaitUntilReached::onStart()
{
  auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  auto tol_res = getInput<double>("tolerance");

  if (!goal_res) {
    RCLCPP_ERROR(node_->get_logger(), "WaitUntilReached: 读取 goal_pose 失败");
    return BT::NodeStatus::FAILURE;
  }

  goal_ = goal_res.value();
  tolerance_ = (tol_res) ? tol_res.value() : 0.5;

  if (!getInput("robot_base_frame", robot_base_frame_)) {
    robot_base_frame_ = "gimbal_yaw_fake";
  }

  RCLCPP_INFO(
    node_->get_logger(), "WaitUntilReached: 等待到达 [%.2f, %.2f], 阈值=%.2f, base_frame=%s",
    goal_.pose.position.x, goal_.pose.position.y, tolerance_, robot_base_frame_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitUntilReached::onRunning()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", robot_base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "WaitUntilReached: TF 查询失败: %s",
      ex.what());
    return BT::NodeStatus::RUNNING;
  }

  double dx = transform.transform.translation.x - goal_.pose.position.x;
  double dy = transform.transform.translation.y - goal_.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance < tolerance_) {
    RCLCPP_INFO(
      node_->get_logger(), "WaitUntilReached: 已到达目标点 (距离=%.2f)", distance);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitUntilReached::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "WaitUntilReached: 被中断");
}

BT::PortsList WaitUntilReached::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "目标位姿"),
    BT::InputPort<double>("tolerance", 0.5, "到达判定距离阈值 (m)"),
    BT::InputPort<std::string>("robot_base_frame", "gimbal_yaw_fake", "机器人基座坐标系"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::WaitUntilReached>("WaitUntilReached");
}
