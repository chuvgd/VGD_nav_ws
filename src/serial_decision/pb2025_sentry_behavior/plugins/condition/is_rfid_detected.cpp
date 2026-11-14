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

#include "pb2025_sentry_behavior/plugins/condition/is_rfid_detected.hpp"

namespace pb2025_sentry_behavior
{

IsRfidDetectedCondition::IsRfidDetectedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsRfidDetectedCondition::checkRfidStatus, this), config)
{
}

BT::NodeStatus IsRfidDetectedCondition::checkRfidStatus()
{
  auto msg = getInput<pb_rm_interfaces::msg::RfidStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RfidStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  bool ally_supply_point = false;
  getInput("ally_supply_point", ally_supply_point);
  return ally_supply_point && msg->ally_supply_point ? BT::NodeStatus::SUCCESS :
                                                       BT::NodeStatus::FAILURE;
}

BT::PortsList IsRfidDetectedCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RfidStatus>(
      "key_port", "{@referee_rfidStatus}", "RfidStatus port on blackboard"),
    BT::InputPort<bool>("ally_supply_point", true, "己方补给点"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsRfidDetectedCondition>("IsRfidDetected");
}
