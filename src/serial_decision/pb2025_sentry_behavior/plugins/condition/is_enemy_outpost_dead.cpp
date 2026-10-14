#include "pb2025_sentry_behavior/plugins/condition/is_enemy_outpost_dead.hpp"

namespace pb2025_sentry_behavior{

IsEnemyOutpostDeadCondition::IsEnemyOutpostDeadCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsEnemyOutpostDeadCondition::checkEnemyOutpost, this), config)
{
}

BT::NodeStatus IsEnemyOutpostDeadCondition::checkEnemyOutpost(){
    int enemy_outpost_flag;
    auto msg = getInput<pb_rm_interfaces::msg::GameRobotHP>("key_port");
    
    if (!msg) {
        RCLCPP_ERROR(logger_, "RobotStatus message is not available");
        return BT::NodeStatus::FAILURE;
    }//空指针异常处理

    getInput("enemy_outpost_flag",enemy_outpost_flag);

    const bool enemy_outpost_flag_ok = (msg->enemy_outpost_flag==enemy_outpost_flag);

    return (enemy_outpost_flag_ok) ? BT::NodeStatus::SUCCESS :BT::NodeStatus::FAILURE;

}

BT::PortsList IsEnemyOutpostDeadCondition::providedPorts(){
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "AllRobotHP port on blackboard"),
    BT::InputPort<int>("enemy_outpost_flag", 1 , "Flag of Outpost. NOTE: 1: dead; 0: survival")
    };//定义输入端口，设置端口相关默认参数值
}

} // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsEnemyOutpostDeadCondition>("IsEnemyOutpostDead");
}

