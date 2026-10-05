#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NEXT_WAYPOINT_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NEXT_WAYPOINT_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace pb2025_sentry_behavior
{
/**
 * @brief 切换到下一个航点索引（单程）
 *
 * 读取当前 wp_idx 和 total_waypoints，递增索引 wp_idx + 1 并写回黑板。
 * 当到达最后一个航点 (next >= total) 时返回 FAILURE，用于配合 KeepRunningUntilFailure
 * 终止导航循环。适合单程路点导航场景——走完所有航点即停止。
 *
 * 端口:
 *   - wp_idx: 双向端口，当前航点索引 (int)，同时读写
 *   - total_waypoints: 航点总数 (int)
 */
class NextWaypoint : public BT::SyncActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name XML 标签中的节点名称
   * @param config BT 节点配置
   */
  NextWaypoint(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief 创建 BT 端口列表
   * @return BT::PortsList 包含节点专用的端口
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief tick 函数：递增 wp_idx（循环）
   * @return BT::NodeStatus 切换结果
   */
  BT::NodeStatus tick() override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NEXT_WAYPOINT_HPP_
