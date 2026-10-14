#include "pb2025_sentry_behavior/plugins/action/switch_sentry_posture.hpp"

namespace pb2025_sentry_behavior {

SwitchSentryPostureAction::SwitchSentryPostureAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
  : BT::RosTopicPubNode<pb_rm_interfaces::msg::SentryPosture>(name, config, params)
{
}

bool SwitchSentryPostureAction::setMessage(pb_rm_interfaces::msg::SentryPosture & msg)
{
  // 1. 从行为树获取目标姿态值
  uint8_t target_posture;
  if (!getInput<uint8_t>("target_posture", target_posture)) {
    RCLCPP_ERROR(logger(), "Missing required input [target_posture]");
    return false;
  }

  // 2. 参数合法性检查 (根据你定义的常量范围 1-3)
  if (target_posture < 1 || target_posture > 6) {
    RCLCPP_ERROR(logger(), "Invalid posture ID: %d. Must be 1 (Attack), 2 (Defense), 3 (Move) , 4 (Powerful_Attack) , 5 (Powerful_Defense) , 6 (Powerful_Move).", target_posture);
    return false;
  }

  // 3. 填充 ROS 2 消息
  msg.current_posture = target_posture;

  // 打印直观的姿态名称，方便调试
  std::string posture_name = (target_posture == 1) ? "ATTACK" : 
                             (target_posture == 2) ? "DEFENSE" : 
                             (target_posture == 3) ? "MOVE" :
                             (target_posture == 4) ? "Powerful_Attack" :
                             (target_posture == 5) ? "Powerful_Defense" :
                             "Powerful_Move";
  RCLCPP_INFO_STREAM(logger(), "Published Sentry Posture Command: [" << posture_name << "] (ID: " << static_cast<int>(target_posture) << ")");

  return true;
}

BT::PortsList SwitchSentryPostureAction::providedPorts()
{
  BT::PortsList additional_ports = {
    // 输入端口：目标姿态值(包括强化姿态)
    BT::InputPort<uint8_t>("target_posture", "Target posture ID ")
  };
  return providedBasicPorts(additional_ports);
}

} // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SwitchSentryPostureAction, "SwitchSentryPosture");