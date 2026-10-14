#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_BB_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_BB_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief 通过黑板值发布 Nav2 目标点（CoD 风格）
 *
 * 与 PubNav2Goal 的区别：
 *   - 输入端口名为 "goal_pose"（而非 "goal"）
 *   - 直接从黑板读取 PoseStamped，不做字符串解析
 *   - 不写回黑板（无 goal_out 输出端口）
 *   - 支持 frame_id、min_pub_interval_ms 端口
 */
class PubNav2GoalBBAction : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
  PubNav2GoalBBAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(geometry_msgs::msg::PoseStamped & msg) override;

private:
  rclcpp::Logger logger() { return node_->get_logger(); }
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_BB_HPP_
