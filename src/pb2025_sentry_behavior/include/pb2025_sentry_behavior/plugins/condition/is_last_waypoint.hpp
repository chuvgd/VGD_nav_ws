#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_LAST_WAYPOINT_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_LAST_WAYPOINT_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace pb2025_sentry_behavior
{

class IsLastWaypointCondition : public BT::SimpleConditionNode
{
public:
  IsLastWaypointCondition(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkIsLastWaypoint();
};

}  // namespace pb2025_sentry_behavior

#endif
