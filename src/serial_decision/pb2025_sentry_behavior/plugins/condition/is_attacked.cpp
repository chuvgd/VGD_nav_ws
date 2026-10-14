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

#include "pb2025_sentry_behavior/plugins/condition/is_attacked.hpp"

namespace pb2025_sentry_behavior
{

IsAttackedCondition::IsAttackedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttackedCondition::checkIsAttacked, this), config)
{
}

BT::NodeStatus IsAttackedCondition::checkIsAttacked()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  uint16_t current_hp = msg->current_hp;
  bool hp_dropped = (current_hp < last_hp_);
  last_hp_ = current_hp;

  // 检查敌人是否可见
  bool no_enemy = true;
  auto vision = getInput<std_msgs::msg::String>("vision_data");
  if (vision && !vision->data.empty()) {
    std::string data = vision->data;
    std::vector<std::string> parts;
    std::stringstream ss(data);
    std::string item;
    while (std::getline(ss, item, ',')) {
      parts.push_back(item);
    }
    if (parts.size() >= 4 &&
        (parts[2] == "1" || parts[2] == "1.0" || parts[2] == "1.000000")) {
      no_enemy = false;
    }
  }

  bool is_attacked = hp_dropped && no_enemy;
  return is_attacked ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsAttackedCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<std_msgs::msg::String>(
      "vision_data", "{@vision_string_data}", "Vision string data port on blackboard")
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsAttackedCondition>("IsAttacked");
}
