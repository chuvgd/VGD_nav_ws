#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_CMD_VEL_ZERO_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_CMD_VEL_ZERO_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior {

class IsCmdVelZeroCondition : public BT::SimpleConditionNode {
public:
  IsCmdVelZeroCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkCmdVel();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsCmdVelZeroCondition");
};

} // namespace pb2025_sentry_behavior

#endif // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_CMD_VEL_ZERO_HPP_