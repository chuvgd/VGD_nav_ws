#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ENEMY_IN_RANGE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ENEMY_IN_RANGE_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pb2025_sentry_behavior
{

class IsEnemyInRangeCondition : public BT::SimpleConditionNode
{
public:
  IsEnemyInRangeCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkEnemyInRange();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsEnemyInRangeCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ENEMY_IN_RANGE_HPP_
