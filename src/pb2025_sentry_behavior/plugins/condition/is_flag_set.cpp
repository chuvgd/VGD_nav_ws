#include "pb2025_sentry_behavior/plugins/condition/is_flag_set.hpp"

namespace pb2025_sentry_behavior
{

IsFlagSetCondition::IsFlagSetCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsFlagSetCondition::checkFlag, this), config)
{
}

BT::NodeStatus IsFlagSetCondition::checkFlag()
{
  std::string flag_name;
  if (!getInput<std::string>("flag_name", flag_name)) {
    return BT::NodeStatus::FAILURE;
  }

  int value = 0;
  if (config().blackboard->get(flag_name, value) && value == 1) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsFlagSetCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>("flag_name", "黑板上要检查的 int 标志名"),
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsFlagSetCondition>("IsFlagSet");
}
