#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SWITCH_FLAG_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SWITCH_FLAG_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "pb_rm_interfaces/msg/switch_flag.hpp"
#include "pb2025_sentry_behavior/pb2025_sentry_behavior_client.hpp"
#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"


namespace pb2025_sentry_behavior {

class SwitchFlagAction : public BT::RosTopicPubNode<pb_rm_interfaces::msg::SwitchFlag> {
public:
  SwitchFlagAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(pb_rm_interfaces::msg::SwitchFlag & msg) override;

private:
  rclcpp::Logger logger() { return node_->get_logger(); }
};

} // namespace pb2025_sentry_behavior

#endif // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SWITCH_FLAG_HPP_