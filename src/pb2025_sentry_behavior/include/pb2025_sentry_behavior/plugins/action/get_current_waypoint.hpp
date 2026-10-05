#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__GET_CURRENT_WAYPOINT_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__GET_CURRENT_WAYPOINT_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief 根据 wp_idx 从航点列表中取出当前目标点
 *
 * 从黑板读取航点列表和当前索引，输出当前目标位姿和等待时间。
 *
 * 输入端口:
 *   - waypoints: 航点列表 (std::vector<geometry_msgs::msg::PoseStamped>)
 *   - wait_times: 等待时间列表 (std::vector<double>)
 *   - wp_idx: 当前航点索引 (int)
 *
 * 输出端口:
 *   - current_goal: 当前目标点 (geometry_msgs::msg::PoseStamped)
 *   - current_wait_sec: 当前航点等待时间 (double)
 */
class GetCurrentWaypoint : public BT::SyncActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name XML 标签中的节点名称
   * @param config BT 节点配置
   */
  GetCurrentWaypoint(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief 创建 BT 端口列表
   * @return BT::PortsList 包含节点专用的端口
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief tick 函数：从列表中提取当前航点
   * @return BT::NodeStatus 提取结果
   */
  BT::NodeStatus tick() override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__GET_CURRENT_WAYPOINT_HPP_
