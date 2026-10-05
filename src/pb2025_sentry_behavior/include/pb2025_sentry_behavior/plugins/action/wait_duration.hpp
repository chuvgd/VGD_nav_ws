#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_DURATION_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_DURATION_HPP_

#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pb2025_sentry_behavior
{

class WaitDuration : public BT::StatefulActionNode
{
public:
  WaitDuration(const std::string & name, const BT::NodeConfig & config);
  ~WaitDuration() override = default;

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  double duration_;
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Logger logger_ = rclcpp::get_logger("WaitDuration");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_DURATION_HPP_
