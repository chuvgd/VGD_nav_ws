#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_WAYPOINTS_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_WAYPOINTS_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief 从 CSV 文件加载航点列表到黑板
 *
 * CSV 格式: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,wait_sec
 * 其中 wait_sec 列可选，缺省为 0（不等待）
 *
 * 输出端口:
 *   - waypoints: 航点列表 (std::vector<geometry_msgs::msg::PoseStamped>)
 *   - wait_times: 每个航点的等待时间 (std::vector<double>)
 *   - total_waypoints: 航点总数 (int)
 */
class LoadWaypoints : public BT::SyncActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name XML 标签中的节点名称
   * @param config BT 节点配置
   */
  LoadWaypoints(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief 创建 BT 端口列表
   * @return BT::PortsList 包含节点专用的端口
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief tick 函数：加载 CSV 文件并写入黑板
   * @return BT::NodeStatus 加载结果
   */
  BT::NodeStatus tick() override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_WAYPOINTS_HPP_
