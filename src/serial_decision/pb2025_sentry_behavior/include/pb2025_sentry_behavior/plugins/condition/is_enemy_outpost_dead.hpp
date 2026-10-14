#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ENEMY_OUTPOST_DEAD_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ENEMY_OUTPOST_DEAD_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief A BT::ConditionNode that get GameStatus from port and
 * returns SUCCESS when current game status and remain time is expected
 */
class IsEnemyOutpostDeadCondition : public BT::SimpleConditionNode
{
public:
  IsEnemyOutpostDeadCondition(const std::string & name, const BT::NodeConfig & config);//构造函数

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();//静态方法，去定义和提供端口

private:
  /**
   * @brief Tick function for game status ports
   */
  BT::NodeStatus checkEnemyOutpost();//核心逻辑，检查机器人状态

  rclcpp::Logger logger_ = rclcpp::get_logger("IsEnemyOutpostDeadCondition");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ENEMY_OUTPOST_DEAD_HPP_

