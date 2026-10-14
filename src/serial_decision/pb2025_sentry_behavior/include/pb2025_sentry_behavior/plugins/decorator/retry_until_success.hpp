#ifndef RETRY_UNTIL_SUCCESS_HPP_
#define RETRY_UNTIL_SUCCESS_HPP_

#include "behaviortree_cpp/decorator_node.h"

namespace pb2025_sentry_behavior {

class RetryUntilSuccess : public BT::DecoratorNode {
 public:
  RetryUntilSuccess(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus tick() override;
};

}  // namespace pb2025_sentry_behavior

#endif  // RETRY_UNTIL_SUCCESS_HPP_