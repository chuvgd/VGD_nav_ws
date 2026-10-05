#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_CMDVEL_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_CMDVEL_
#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace pb2025_sentry_behavior
{

class PublishCmdVelAction : public BT::RosTopicPubStatefulActionNode<geometry_msgs::msg::Twist>//定义PublishTwistAction继承自BT::RosTopicPubStatefulActionNode模板父类，模板参数geometry_msgs::msg::Twist
//StatefulActionNode：这个父类多了“状态”管理，允许节点在被中断（Halt）时，发送一个特定的“停止消息”（比如速度为 0）
{
public:
  PublishCmdVelAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);//构造函数

  static BT::PortsList providedPorts();//静态方法定义端口

  bool setMessage(geometry_msgs::msg::Twist & msg) override;//当节点执行时，框架会调用setMessage，让你填充要发送的速度消息，返回true

  bool setHaltMessage(geometry_msgs::msg::Twist & msg) override;//当行为树逻辑改变，或者这个节点被父节点中断时，框架会调用这个函数

private:
  rclcpp::Logger logger() { return node_->get_logger(); }  
};

}  // namespace pb2025_sentry_behavior

#endif //PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_CMDVEL_