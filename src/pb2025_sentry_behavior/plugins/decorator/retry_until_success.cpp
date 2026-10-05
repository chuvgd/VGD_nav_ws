#include "pb2025_sentry_behavior/plugins/decorator/retry_until_success.hpp"

namespace pb2025_sentry_behavior {

RetryUntilSuccess::RetryUntilSuccess(const std::string& name, const BT::NodeConfig& config)
    : BT::DecoratorNode(name, config) {
}

BT::PortsList RetryUntilSuccess::providedPorts() {
  return {};
}

BT::NodeStatus RetryUntilSuccess::tick() {
  // 执行子节点
  auto child_status = child()->executeTick();
  
  setStatus(child_status);
  
  switch (child_status) {
    case BT::NodeStatus::SUCCESS: {
      // 子节点成功，返回成功
      return BT::NodeStatus::SUCCESS;
    }
    
    case BT::NodeStatus::FAILURE: {
      // 子节点失败，继续重试（返回 RUNNING）
      return BT::NodeStatus::RUNNING;
    }
    
    case BT::NodeStatus::RUNNING:
    default: {
      // 子节点运行中，继续等待
      return BT::NodeStatus::RUNNING;
    }
  }
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<pb2025_sentry_behavior::RetryUntilSuccess>("RetryUntilSuccess");
}