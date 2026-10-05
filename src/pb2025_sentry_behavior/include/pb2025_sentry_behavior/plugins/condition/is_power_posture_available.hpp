#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_POWER_POSTURE_AVAILABLE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_POWER_POSTURE_AVAILABLE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace pb2025_sentry_behavior
{

class IsPowerPostureAvailableCondition : public BT::SimpleConditionNode
{
public:
  IsPowerPostureAvailableCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkPowerPostureAvailable();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsPowerPostureAvailableCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_POWER_POSTURE_AVAILABLE_HPP_
