#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

class IsGoalReachedCondition : public BT::ConditionNode
{
public:
  IsGoalReachedCondition(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);//处理收到话题的回调函数

  rclcpp::Node::SharedPtr node_;
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;//订阅的目标变量
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;//tf缓冲区，将监听到的tf数据写进缓冲区
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;//tf监听器，监听写入的坐标系

  // std::mutex goal_mutex_;
  // geometry_msgs::msg::PoseStamped latest_goal_;//最后收到的目标
  // bool goal_received_{false};//对于是否收到目标（默认false）

  rclcpp::Logger logger_ = rclcpp::get_logger("IsGoalReached");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_