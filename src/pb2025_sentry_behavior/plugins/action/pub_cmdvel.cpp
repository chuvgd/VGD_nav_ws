#include "pb2025_sentry_behavior/plugins/action/pub_cmdvel.hpp"

namespace pb2025_sentry_behavior
{

PublishCmdVelAction::PublishCmdVelAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)//构造函数：初始化列表
{
}

BT::PortsList PublishCmdVelAction::providedPorts()
{
  return providedBasicPorts(
    {BT::InputPort<double>("v_x", 0.0, "Linear X velocity (m/s)"),
     BT::InputPort<double>("v_y", 0.0, "Linear Y velocity (m/s)"),
     BT::InputPort<double>("v_yaw", 0.0, "Angular Z velocity (rad/s)"),
     BT::InputPort<int>("wp_idx", "当前航点索引"),
     BT::InputPort<int>("total_waypoints", "航点总数"),
    });
}

bool PublishCmdVelAction::setMessage(geometry_msgs::msg::Twist & msg)
{
  int wp_idx;
  int total_waypoints;

  if (!getInput<int>("wp_idx", wp_idx)) {
    RCLCPP_ERROR(logger(), "Missing required input [wp_idx]");
    return false;
  }

  if (!getInput<int>("total_waypoints", total_waypoints)) {
    RCLCPP_ERROR(logger(), "Missing required input [total_waypoints]");
    return false;
  }
  if (wp_idx == total_waypoints - 1){
    double vx = 0.0, vy = 0.0, vyaw = 0.0;
    getInput("v_x", vx);
    getInput("v_y", vy);
    getInput("v_yaw", vyaw);//把从xml文件里面发的速度传入给端口--getinput相当于从xml/黑板->cpp文件

    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.angular.z = vyaw;//把从端口输入给cpp文件的速度数据修改msg，消息填充成功返回true，最后父类（RosTopicPubStatefulActionNode）才会拿着填好的 msg 去执行真正的发布操作（publisher_->publish(msg)）
  
    return true;
  }

  return false;
}

bool PublishCmdVelAction::setHaltMessage(geometry_msgs::msg::Twist & msg)
{
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.angular.z = 0;
  return true;//安全机制，被中断后发0实现安全
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishCmdVelAction, "PublishCmdVel");//注册插件，实现xml文件实例化
