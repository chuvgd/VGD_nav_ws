#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SWITCH_SENTRY_POSTURE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SWITCH_SENTRY_POSTURE_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "pb_rm_interfaces/msg/sentry_posture.hpp"

namespace pb2025_sentry_behavior {

class SwitchSentryPostureAction : public BT::RosTopicPubNode<pb_rm_interfaces::msg::SentryPosture> {
public:
  SwitchSentryPostureAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(pb_rm_interfaces::msg::SentryPosture & msg) override;

private:
  rclcpp::Logger logger() { return node_->get_logger(); }
};

} // namespace pb2025_sentry_behavior

#endif // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SWITCH_SENTRY_POSTURE_HPP_