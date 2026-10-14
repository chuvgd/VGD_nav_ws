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

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

#include <filesystem>
#include <fstream>

#include "std_msgs/msg/string.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"//引入这些自定义消息类型的接口
namespace pb2025_sentry_behavior
{

template <typename T>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  auto sub = node()->create_subscription<T>(
    topic, qos,//topic话题名称，qos服务质量
    [this, bb_key](const typename T::SharedPtr msg) { globalBlackboard()->set(bb_key, *msg); });
  subscriptions_.push_back(sub);
}//模板函数
//详细解释：创建一个订阅者，订阅类型T（模板函数）的消息类型的函数，对于lambda函数--回调函数每次订阅到话题之后执行这个函数，[]捕获列表，匿名函数可以直接使用this（当前实例对象）和bb_key（黑板键名），const typename T::SharedPtr msg--参数，传入ros2的话题指针，{ globalBlackboard()->set(bb_key, *msg); }执行体，在全局黑板写入字典--bb_key：*msg
SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)
{
  node()->declare_parameter("use_cout_logger", false);//声明参数
  node()->get_parameter("use_cout_logger", use_cout_logger_);//从use_cout_logger_获取参数

  subscribe<pb_rm_interfaces::msg::EventData>("referee/event_data", "referee_eventData");
  subscribe<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", "referee_allRobotHP");
  subscribe<pb_rm_interfaces::msg::GameStatus>("referee/game_status", "referee_gameStatus");
  subscribe<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");
  subscribe<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", "referee_rfidStatus");
  subscribe<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", "referee_robotStatus");
  subscribe<pb_rm_interfaces::msg::Buff>("referee/buff", "referee_buff");//在构造函数中引入模板函数subscribe，传入参数topic和bb_key
  subscribe<std_msgs::msg::String>("auto_aim_target_pos","vision_string_data");
  // 订阅 cmd_vel 用于检查速度是否为零
  subscribe<geometry_msgs::msg::Twist>("cmd_vel", "cmd_vel");

  // auto detector_qos = rclcpp::SensorDataQoS();
  // subscribe<auto_aim_interfaces::msg::Armors>("detector/armors", "detector_armors", detector_qos);
  // auto tracker_qos = rclcpp::SensorDataQoS();
  // subscribe<auto_aim_interfaces::msg::Target>("tracker/target", "tracker_target", tracker_qos);//视觉和自瞄数据（高频传感器）--qos策略（留坑）--这里rclcpp::SensorDataQoS()是保持最新消息

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();//rclcpp::KeepLast(1)：队列深度为1；transient_local()：暂时保留/本地持久化；reliable()：可靠传递--必须把消息传到
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);//订阅话题，把收到的话题的数据传入全局的黑板的键中--字典键值对模式
  //globalBlackboard()->set("wp_idx",0);
  //globalBlackboard()->set("home_wp_idx",0);
  //globalBlackboard()->set("patrol_wp_idx",0);
}//构造函数

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}//收到行为树（启动前--配置阶段）

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
}//创建行为树（tick_count_次数初始化为0）--每次创建树或者重新开始任务的时候，把计数器清零

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  return std::nullopt;
}//循环中（即在执行行为树中）--每完成一次行为树逻辑tick_count_加1，返回std::nullopt表示不要打断行为树（无论成功或者失败）

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  logger_cout_.reset();//销毁logger_cout_(不再记录日志)
  std::string result = treeName() +
                       " tree completed with status=" + std::to_string(static_cast<int>(status)) +
                       " after " + std::to_string(tick_count_) + " ticks";//std::to_string转化成字符串去拼接
  return result;
}//结束收尾工作，返回结果

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryBehaviorServer>(options);//创建服务类对象

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());//多线程执行器

  // Groot2 editor requires a model of your registered Nodes.
  // You don't need to write that by hand, it can be automatically
  // generated using the following command.
  std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());

  // Save the XML models to a file
  std::ofstream file(std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml");
  file << xml_models;//自动保存行为树节点模型xml文件--把所有“已注册的C++节点”自动导出成一个 XML 文件

  rclcpp::shutdown();
}
