#include "pb2025_sentry_behavior/plugins/condition/is_goal_reached.hpp"

namespace pb2025_sentry_behavior
{

IsGoalReachedCondition::IsGoalReachedCondition(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config)
{
  auto nh = params.nh.lock();
  if (!nh) {
    throw std::runtime_error{"ROS node is not available"};
  }
  node_ = nh;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}//构造函数对于ros节点的初始化（留坑后续再研究）

// void IsGoalReachedCondition::goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(goal_mutex_);//保证线程安全（留坑后续再研究）
//   latest_goal_ = *msg;//把传入的msg这个智能指针指向的消息内容赋值给latest_goal_这个变量
//   goal_received_ = true;//传入目标话题的消息，设置goal_received_为true，表示已经收到话题
// }

BT::NodeStatus IsGoalReachedCondition::tick()//tick()函数的逻辑
{
  std::string global_frame, robot_base_frame;
  double xy_tolerance;//声明变量

  if (!getInput("global_frame", global_frame)) {
    global_frame = "map";
  }
  if (!getInput("robot_base_frame", robot_base_frame)) {
    robot_base_frame = "gimbal_yaw_fake";
  }
  if (!getInput("xy_tolerance", xy_tolerance)) {
    xy_tolerance = 0.3;
  }//tick()函数处理获取输入端口的值如果没有端口输入就给默认值

  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal_in");
  if (!goal) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(global_frame, robot_base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000, "TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }//监听tf变化，然后把tf变化赋值给transform这个变量，将global_frame这个变量转换为robot_base_frame这个变量的坐标，如果转换失败，返回失败

  double dx = goal->pose.position.x - transform.transform.translation.x;
  double dy = goal->pose.position.y - transform.transform.translation.y;
  double dist = std::hypot(dx, dy);//计算目标点和机器人当前位置的距离，就是2d下的欧式距离

  if (dist <= xy_tolerance) {
    return BT::NodeStatus::SUCCESS;
  }//如果小于等于xy_tolerance，返回成功
  return BT::NodeStatus::FAILURE;
}//否则返回失败

BT::PortsList IsGoalReachedCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>("global_frame", "map", "Global frame for TF lookup"),
    BT::InputPort<std::string>(
      "robot_base_frame", "gimbal_yaw_fake", "Robot base frame for TF lookup"),
    BT::InputPort<double>("xy_tolerance", 0.3, "XY tolerance in meters to consider goal reached"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_in", "{goal_out}", "Goal from blackboard"),//提供输入端口，从黑板中获取目标点，并设置默认值为{goal_out}，表示从黑板中获取目标点，描述为"Goal from blackboard"（输入端口是将节点处理的结果写入黑板）
  };
}//提供输入端口并设置默认值和描述

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::IsGoalReachedCondition, "IsGoalReached");//注册IsGoalReachedCondition这个节点，节点名称是IsGoalReached

//本质是一个反序列化过程，对于xml文件中是以一种序列化的方式进行描述，运行时内存（被反序列化之后）将各变量的值赋值给对应的变量
//BT_REGISTER_NODES / CreateRosNodePlugin 就是告诉反序列化器：遇到标签名 IsGoalReached → 找 libis_goal_reached.so → 调 IsGoalReachedCondition(name, config, params) 构造对象
//这种反序列器机制（后续留坑处理）