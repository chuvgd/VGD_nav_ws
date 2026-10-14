// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{
class PubNav2GoalAction : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>//继承模板基类RosTopicPubNode，模板参数：geometry_msgs::msg::PoseStamped
{
public:
  PubNav2GoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();//静态方法：定义端口

  bool setMessage(geometry_msgs::msg::PoseStamped & goal) override;//作用：当行为树执行到这个节点时，基类会调用这个函数，让你把具体的坐标数据填入 goal 消息中
  // 工作流程：
  //   节点从行为树的输入端口（InputPort）读取坐标字符串（比如 "4.65;-3.5;0"）--输入端口：节点读取参数（xml/黑板->节点）；--输出端口：节点输出数据（节点->黑板）
  //   解析这个字符串，提取出 x, y 和 yaw（偏航角）
  //   将这些值赋给 goal.pose.position.x 等字段
  //   返回 true 表示填充成功，消息会被发送；返回 false 表示失败

private:
  rclcpp::Logger logger() { return node_->get_logger(); }
  rclcpp::Time now() { return node_->now(); }
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_
