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

#include "pb2025_sentry_behavior/plugins/condition/is_detect_enemy.hpp"

namespace pb2025_sentry_behavior
{

IsDetectEnemyCondition::IsDetectEnemyCondition(
  const std::string & name, const BT::NodeConfig & config)
// 将原本的 checkEnemy 绑定改为新的 checkEnemyString 绑定
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyCondition::checkEnemyString, this), config)
{
}

BT::PortsList IsDetectEnemyCondition::providedPorts()
{
  return {
    // 修改端口：读取 std_msgs::msg::String 类型
    // 默认键名改为你在 Server 中设置的 "vision_string_data"
    BT::InputPort<std_msgs::msg::String>(
      "key_port", "{@vision_string_data}", "Vision string data port on blackboard")
  };
}

// 核心逻辑修改为字符串解析
BT::NodeStatus IsDetectEnemyCondition::checkEnemyString()
{
  // 1. 从黑板获取字符串数据（读取 std_msgs::msg::String 类型）
  auto msg = getInput<std_msgs::msg::String>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "Vision string message is not available: %s", msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 2. 获取消息中的字符串内容
  std::string current_data = msg.value().data;

  RCLCPP_DEBUG(logger_, "Received vision data: '%s'", current_data.c_str());

  // 2. 如果没有收到数据，返回 FAILURE
  if (current_data.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  try {
    std::vector<std::string> parts;
    std::stringstream ss(current_data);
    std::string item;
    
    // 3. 按逗号分割字符串
    while (std::getline(ss, item, ',')) {
      parts.push_back(item);
    }

    // 4. 检查字段数量是否符合预期 (x,y,z,id 共4个)
    if (parts.size() < 4) {
      return BT::NodeStatus::FAILURE;
    }

    // 5. 提取第3个字段 (索引为2)，判断是否为 "1"
    // 对应格式：有目标 x,y,1,id
    RCLCPP_DEBUG(logger_, "Detection flag: '%s'", parts[2].c_str());
    
    // 支持 "1"、"1.0"、"1.000000" 等格式
    if (parts[2] == "1" || parts[2] == "1.0" || parts[2] == "1.000000") {
      RCLCPP_INFO(logger_, "Enemy detected!");
      return BT::NodeStatus::SUCCESS; // 检查到敌人
    } else if (parts[2] == "0" || parts[2] == "0.0" || parts[2] == "0.000000") {
      RCLCPP_DEBUG(logger_, "No enemy detected");
      return BT::NodeStatus::FAILURE; // 无敌人
    } else {
      RCLCPP_WARN(logger_, "Invalid detection flag: '%s'", parts[2].c_str());
      return BT::NodeStatus::FAILURE; // 无效数据
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Parse error: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDetectEnemyCondition>("IsDetectEnemy");
}