#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_UNTIL_REACHED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_UNTIL_REACHED_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{
/**
 * @brief 通过 TF2 查询机器人位姿，判断是否到达目标点
 *
 * 该节点持续查询 map -> robot_base_frame 的 TF 变换，计算与目标点的欧氏距离。
 * 当距离小于 tolerance 阈值时返回 SUCCESS，否则保持 RUNNING。
 *
 * 输入端口:
 *   - goal_pose: 目标位姿 (geometry_msgs::msg::PoseStamped)
 *   - tolerance: 到达判定距离阈值，默认 0.5m (double)
 *   - robot_base_frame: 机器人基座坐标系，默认 "gimbal_yaw_fake" (std::string)
 */
class WaitUntilReached : public BT::StatefulActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name XML 标签中的节点名称
   * @param config BT 节点配置
   */
  WaitUntilReached(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief 析构函数
   */
  ~WaitUntilReached() override = default;

  /**
   * @brief 创建 BT 端口列表
   * @return BT::PortsList 包含节点专用的端口
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief 节点首次 tick 时调用，初始化目标位姿和阈值
   * @return BT::NodeStatus RUNNING 或 FAILURE
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief 节点处于 RUNNING 状态时的 tick 逻辑
   * @return BT::NodeStatus RUNNING（未到达）或 SUCCESS（已到达）
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief 节点被中断时调用
   */
  void onHalted() override;

  // ROS 2 内部节点，用于 TF2 查询
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped goal_;
  double tolerance_;
  std::string robot_base_frame_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_UNTIL_REACHED_HPP_
