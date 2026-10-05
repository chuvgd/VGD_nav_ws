#include "pb2025_sentry_behavior/plugins/decorator/delay_after_success.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior {

DelayAfterSuccess::DelayAfterSuccess(const std::string& name, const BT::NodeConfig& config)
    : BT::DecoratorNode(name, config), delay_started_(false) {
  getInput("delay_ms", delay_ms_);
  RCLCPP_INFO(rclcpp::get_logger("DelayAfterSuccess"), 
              "Created with delay: %d ms", delay_ms_);
}

BT::PortsList DelayAfterSuccess::providedPorts() {
  return {
    BT::InputPort<int>("delay_ms", 1000, "Delay after success (ms)")
  };
}

BT::NodeStatus DelayAfterSuccess::tick() {
  // 获取子节点状态
  auto child_status = child()->executeTick();
  
  setStatus(child_status);
  
  switch (child_status) {
    case BT::NodeStatus::SUCCESS: {
      if (!delay_started_) {
        // 第一次成功，记录开始时间
        start_time_ = std::chrono::steady_clock::now();
        delay_started_ = true;
        return BT::NodeStatus::RUNNING;
      }
      
      // 检查是否已经等待足够时间
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_time_);
      
      if (elapsed.count() >= delay_ms_) {
        // 延迟完成，返回成功
        delay_started_ = false;
        return BT::NodeStatus::SUCCESS;
      }
      
      // 继续等待
      return BT::NodeStatus::RUNNING;
    }
    
    case BT::NodeStatus::FAILURE: {
      // 子节点失败，重置延迟状态
      delay_started_ = false;
      return BT::NodeStatus::FAILURE;
    }
    
    case BT::NodeStatus::RUNNING:
    default: {
      // 子节点运行中，重置延迟状态
      delay_started_ = false;
      return BT::NodeStatus::RUNNING;
    }
  }
}

}  // namespace pb2025_sentry_behavior


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<pb2025_sentry_behavior::DelayAfterSuccess>("DelayAfterSuccess");
}


//记住了：函数参数是输入值，外部输入！（主函数）
//类的构造函数初始化列表是先调用父类构造函数，把外部传给构造函数的参数传递给父类，然后再初始化子类的成员函数