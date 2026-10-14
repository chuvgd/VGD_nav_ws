#ifndef DELAY_AFTER_SUCCESS_HPP_
#define DELAY_AFTER_SUCCESS_HPP_

#include "behaviortree_cpp/decorator_node.h"
#include <chrono>

namespace pb2025_sentry_behavior {

class DelayAfterSuccess : public BT::DecoratorNode {
 public:
  DelayAfterSuccess(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus tick() override;
  
  std::chrono::steady_clock::time_point start_time_;
  bool delay_started_{false};
  int delay_ms_{1000};
};

}  // namespace pb2025_sentry_behavior

#endif  // DELAY_AFTER_SUCCESS_HPP_