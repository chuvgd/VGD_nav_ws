#include "pb2025_sentry_behavior/plugins/condition/is_cmd_vel_zero.hpp"

namespace pb2025_sentry_behavior {

IsCmdVelZeroCondition::IsCmdVelZeroCondition(const std::string & name, const BT::NodeConfig & config)
  : BT::SimpleConditionNode(name, std::bind(&IsCmdVelZeroCondition::checkCmdVel, this), config)
{
}

BT::NodeStatus IsCmdVelZeroCondition::checkCmdVel()
{
  // 1. 从行为树黑板获取 cmd_vel 消息
  auto msg = getInput<geometry_msgs::msg::Twist>("cmd_vel");
  
  if (!msg) {
    RCLCPP_ERROR(logger_, "CmdVel message is not available on the blackboard");
    return BT::NodeStatus::FAILURE;
  }

  // 2. 获取阈值参数（用于处理浮点数精度误差）
  // 如果未设置，使用默认值 0.05
  double linear_threshold = 0.05;
  getInput("linear_threshold", linear_threshold); 

  // 3. 提取 vx 和 vy 并取绝对值
  // 完全忽略 angular.z (角速度)，只关心底盘平面移动
  double vx = std::abs(msg->linear.x);
  double vy = std::abs(msg->linear.y);

  // 4. 判断逻辑：只有当 vx 和 vy 都小于等于阈值时，才认为是 "Zero"
  bool is_vx_zero = (vx <= linear_threshold);
  bool is_vy_zero = (vy <= linear_threshold);

  // 5. 调试日志（可选）
  RCLCPP_DEBUG_STREAM(logger_, "CmdVel Check (XY Only) - Vx: " << vx 
                          << " | Vy: " << vy 
                          << " | Threshold: " << linear_threshold);

  // 6. 返回结果
  if (is_vx_zero && is_vy_zero) {
    return BT::NodeStatus::SUCCESS; // 满足条件：底盘没有平面移动
  } else {
    return BT::NodeStatus::FAILURE; // 不满足：底盘正在前后或左右移动
  }
}

BT::PortsList IsCmdVelZeroCondition::providedPorts()
{
  return {
    // 输入端口：获取速度指令
    BT::InputPort<geometry_msgs::msg::Twist>(
      "cmd_vel", 
      "{@cmd_vel}", 
      "The cmd_vel command from controller or behavior tree output"),

    // 输入端口：线速度阈值 (同时作用于 vx 和 vy)
    // 默认设为 0.05 m/s，比这个值小就认为是 0
    BT::InputPort<double>(
      "linear_threshold", 
      0.05, 
      "Threshold below which linear velocity (vx/vy) is considered zero")
  };
}

} // namespace pb2025_sentry_behavior

// 注册节点
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsCmdVelZeroCondition>("IsCmdVelZero");
}