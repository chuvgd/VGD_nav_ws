#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_IN_SEMANTIC_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_IN_SEMANTIC_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "pb2025_sentry_behavior/tools/semantic_regions.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{
class IsInSemanticRegion : public BT::ConditionNode
{
public:
  IsInSemanticRegion(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  tf2_ros::Buffer tf_buffer_;

  tf2_ros::TransformListener tf_listener_;

  SemanticRegions regions_;

  std::string frame_;
  
  bool valid_ = false;
};
}

#endif