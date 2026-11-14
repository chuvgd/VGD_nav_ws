#include "pb2025_sentry_behavior/plugins/action/wait_duration.hpp"

namespace pb2025_sentry_behavior
{

WaitDuration::WaitDuration(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config), duration_(0.0)
{
}

BT::NodeStatus WaitDuration::onStart()
{
  auto dur_res = getInput<double>("duration_sec");
  duration_ = (dur_res) ? dur_res.value() : 0.0;

  if (duration_ <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }

  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger_, "开始等待 %.1f 秒", duration_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitDuration::onRunning()
{
  auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (elapsed >= duration_) {
    RCLCPP_INFO(logger_, "等待 %.1f 秒完成", duration_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitDuration::onHalted()
{
  RCLCPP_WARN(logger_, "等待被中断");
}

BT::PortsList WaitDuration::providedPorts()
{
  return {
    BT::InputPort<double>("duration_sec", 5.0, "等待时长 (秒), 0 则跳过"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::WaitDuration>("WaitDuration");
}
