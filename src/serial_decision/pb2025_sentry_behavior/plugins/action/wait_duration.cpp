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
  // 读取视觉数据, 如果检测到敌人则重置计时器
  auto vision = getInput<std_msgs::msg::String>("key_port");
  if (vision && !vision->data.empty()) {
    std::string data = vision->data;
    std::vector<std::string> parts;
    std::stringstream ss(data);
    std::string item;
    while (std::getline(ss, item, ',')) {
      parts.push_back(item);
    }
    // vision 格式: x,y,flag,id
    if (parts.size() >= 4 &&
        (parts[2] == "1" || parts[2] == "1.0" || parts[2] == "1.000000")) {
      start_time_ = std::chrono::steady_clock::now();
      RCLCPP_DEBUG(logger_, "检测到敌人, 重置等待计时器");
      return BT::NodeStatus::RUNNING;
    }
  }

  auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (elapsed >= duration_) {
    RCLCPP_INFO(logger_, "连续 %.1f 秒无敌人, 等待完成", duration_);
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
    BT::InputPort<double>("duration_sec", 5.0, "连续无敌人的等待时长 (秒), 0 则跳过"),
    BT::InputPort<std_msgs::msg::String>(
      "key_port", "{@vision_string_data}", "Vision string data port on blackboard"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::WaitDuration>("WaitDuration");
}
