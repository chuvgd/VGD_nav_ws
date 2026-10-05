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

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"

#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)//构造函数（初始化列表先调用父类构造函数）--父类会自动处理 ROS 2 发布者的创建（create_publisher）
                                                                      //name：节点在行为树 XML 中的名字。
                                                                      // conf：节点的配置信息（比如黑板指针）
                                                                      // params：ROS 2 相关的参数（比如节点句柄、话题名称等）
{
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");//端口名为goal
                                                                //能够自动识别并解析 XML 中的字符串（getInput 是节点与外部（XML 或 黑板）进行交互的核心方式--拿外部数据）
                                                                //setOutput 就是节点“把处理结果递出去”（节点->黑板）

  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.pose = goal->pose;//修改msg--就是相当于引用传递把goal写入msg
  setOutput("goal_out", msg);//把传入的msg写入黑板，端口名为"goal_out"，实际上是把msg写入{goal_out}，如果xml中没有写明黑板键名，默认黑板键名为{goal_out}
  return true;//消息填充成功返回true，父类RosTopicPubNode拿着msg去执行发布操作
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_out", "{goal_out}","Published goal pose"),//提供输出端口，定义输出端口名为"goal_out"，默认键名为{goal_out}，描述为"Published goal pose"（输出端口是将节点处理的结果写入黑板）
  };//定义端口列表
    // 声明了一个名为 "goal" 的输入端口，类型是 PoseStamped
    // 默认值 "0;0;0"：这里利用了字符串转换机制，如果 XML 没传值，就默认去原点
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"//插件宏定义头文件
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");//注册插件
//告诉行为树工厂：“有一个叫 PubNav2Goal 的节点，对应的 C++ 类是 pb2025_sentry_behavior::PubNav2GoalAction”
//这样你在 XML 里写 <PubNav2Goal .../> 时，程序才知道要实例化这个类--xml描述文件实际上是去实例化这个类



//只有发点操作，setMessage()填充消息然后返回true然后父类RosTopicPubNode拿着msg去执行发布操作（父类 RosTopicPubNode 会自动创建 ROS 发布者）
//留坑，整个行为树流程梳理，从写入黑板值开始到端口拿到黑板值，最后实现逻辑