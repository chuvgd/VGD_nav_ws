#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_FLAG_SET_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_FLAG_SET_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace pb2025_sentry_behavior
{

class IsFlagSetCondition : public BT::SimpleConditionNode
{
public:
  IsFlagSetCondition(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkFlag();
};

}  // namespace pb2025_sentry_behavior

#endif
