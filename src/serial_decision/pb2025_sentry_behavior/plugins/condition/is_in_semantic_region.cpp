#include "pb2025_sentry_behavior/plugins/condition/is_in_semantic_region.hpp"

namespace pb2025_sentry_behavior
{
IsInSemanticRegion::IsInSemanticRegion(const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
  : BT::ConditionNode(name, config), node_(params.nh.lock()), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
  {
    if (!node_->has_parameter("semantic_map_file")) {
      node_->declare_parameter<std::string>("semantic_map_file", "");
    }

    if (!node_->has_parameter("allowed_regions")) {
      node_->declare_parameter<std::vector<std::string>>("allowed_regions", std::vector<std::string>{});
    }

    if (!node_->has_parameter("semantic_robot_frame")) {
      node_->declare_parameter<std::string>("semantic_robot_frame", "chassis");
    }

    if (!node_->has_parameter("semantic_enabled")) {
      node_->declare_parameter<bool>("semantic_enabled", false);
    }

    const auto path = node_->get_parameter("semantic_map_file").as_string();

    const auto ids = node_->get_parameter("allowed_regions").as_string_array();

    frame_ = node_->get_parameter("semantic_robot_frame").as_string();

    valid_ = node_->get_parameter("semantic_enabled").as_bool();

    if (valid_) {
      std::string error;

      valid_ = regions_.load(path, ids, error);

      if (!valid_) RCLCPP_WARN(node_->get_logger(), "Pursuit disabled: %s", error.c_str());
    }
  }

BT::PortsList IsInSemanticRegion::providedPorts() { return {}; }

BT::NodeStatus IsInSemanticRegion::tick()
{
    if (!valid_) return BT::NodeStatus::FAILURE;

    try {
      const auto pose = tf_buffer_.lookupTransform("map", frame_, tf2::TimePointZero);

      return regions_.contains(pose.transform.translation.x, pose.transform.translation.y) ?
        BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Region TF unavailable: %s", ex.what());
      
      return BT::NodeStatus::FAILURE;
    }
  }
};

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::IsInSemanticRegion, "IsInSemanticRegion");