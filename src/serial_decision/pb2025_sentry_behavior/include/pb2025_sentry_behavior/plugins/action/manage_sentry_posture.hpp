#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__MANAGE_SENTRY_POSTURE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__MANAGE_SENTRY_POSTURE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/sentry_posture.hpp"
#include "std_msgs/msg/string.hpp"

namespace pb2025_sentry_behavior
{

uint8_t selectSentryPosture(
  const pb_rm_interfaces::msg::RobotStatus & status, const std::string & vision,
  bool vision_fresh, bool recovery);

class ManageSentryPostureAction : public BT::SyncActionNode
{
public:
  ManageSentryPostureAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<pb_rm_interfaces::msg::SentryPosture>::SharedPtr publisher_;
  
  uint8_t last_posture_ = 0;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__MANAGE_SENTRY_POSTURE_HPP_
