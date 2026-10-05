#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class AutoHighGroundNavigator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    explicit AutoHighGroundNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("auto_high_ground_navigator", options)
    {
        // 声明参数
        this->declare_parameter<bool>("enabled", false);
        this->declare_parameter<double>("high_ground_x", 5.0);
        this->declare_parameter<double>("high_ground_y", 5.0);
        this->declare_parameter<double>("high_ground_z", 0.0);
        this->declare_parameter<double>("high_ground_orientation_w", 1.0);
        this->declare_parameter<double>("delay_before_start", 5.0);
        this->declare_parameter<bool>("auto_start", true);
        this->declare_parameter<std::string>("action_server_name", "/red_standard_robot1/navigate_to_pose");  // 修正这里！
        this->declare_parameter<double>("server_timeout", 10.0);
        this->declare_parameter<int>("max_retry_attempts", 3);
        
        // 获取参数
        enabled_ = this->get_parameter("enabled").as_bool();
        high_ground_x_ = this->get_parameter("high_ground_x").as_double();
        high_ground_y_ = this->get_parameter("high_ground_y").as_double();
        high_ground_z_ = this->get_parameter("high_ground_z").as_double();
        high_ground_orientation_w_ = this->get_parameter("high_ground_orientation_w").as_double();
        delay_before_start_ = this->get_parameter("delay_before_start").as_double();
        auto_start_ = this->get_parameter("auto_start").as_bool();
        action_server_name_ = this->get_parameter("action_server_name").as_string();
        server_timeout_ = this->get_parameter("server_timeout").as_double();
        max_retry_attempts_ = this->get_parameter("max_retry_attempts").as_int();
        
        RCLCPP_INFO(this->get_logger(), "=== 自动高地导航节点 ===");
        RCLCPP_INFO(this->get_logger(), "动作服务器名称: %s", action_server_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "服务器超时时间: %.1f 秒", server_timeout_);
        RCLCPP_INFO(this->get_logger(), "最大重试次数: %d", max_retry_attempts_);
        RCLCPP_INFO(this->get_logger(), "高地坐标: (%.2f, %.2f, %.2f)", 
                   high_ground_x_, high_ground_y_, high_ground_z_);
        RCLCPP_INFO(this->get_logger(), "延迟启动: %.1f 秒", delay_before_start_);
        
        // 创建动作客户端
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            action_server_name_
        );
        
        if (!nav_client_) {
            RCLCPP_FATAL(this->get_logger(), "动作客户端创建失败！");
            return;
        }
        
        // 检查动作服务器是否可用
        checkServerAvailability();
        
        // 创建服务
        start_service_ = this->create_service<std_srvs::srv::SetBool>(
            "start_high_ground_navigation",
            std::bind(&AutoHighGroundNavigator::startServiceCallback, this, _1, _2)
        );
        
        // 创建状态发布者
        status_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "high_ground_navigation_status",
            10
        );
        
        // 如果启用且自动开始，启动定时器
        if (enabled_ && auto_start_) {
            RCLCPP_INFO(this->get_logger(), "自动高地导航已启用，将在 %.1f 秒后开始", delay_before_start_);
            
            auto timer_callback = [this]() -> void {
                this->navigateToHighGround();
                // 只执行一次
                if (start_timer_) {
                    start_timer_->cancel();
                    start_timer_.reset();
                }
            };
            
            start_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(delay_before_start_),
                timer_callback
            );
        } else if (enabled_) {
            RCLCPP_INFO(this->get_logger(), "高地导航已启用，等待手动启动");
        } else {
            RCLCPP_INFO(this->get_logger(), "高地导航未启用");
        }
    }
    
private:
    // 参数
    bool enabled_;
    double high_ground_x_;
    double high_ground_y_;
    double high_ground_z_;
    double high_ground_orientation_w_;
    double delay_before_start_;
    bool auto_start_;
    std::string action_server_name_;
    double server_timeout_;
    int max_retry_attempts_;
    
    // ROS2组件
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    
    // 检查服务器可用性
    void checkServerAvailability()
    {
        RCLCPP_INFO(this->get_logger(), "检查动作服务器 '%s' 是否可用...", action_server_name_.c_str());
        
        int attempts = 0;
        while (rclcpp::ok() && attempts < max_retry_attempts_) {
            attempts++;
            
            if (nav_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_))) {
                RCLCPP_INFO(this->get_logger(), "✅ 动作服务器连接成功！");
                return;
            } else {
                RCLCPP_WARN(this->get_logger(), 
                           "尝试 %d/%d: 动作服务器未响应", 
                           attempts, max_retry_attempts_);
                
                if (attempts < max_retry_attempts_) {
                    RCLCPP_INFO(this->get_logger(), "等待2秒后重试...");
                    std::this_thread::sleep_for(2s);
                }
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), 
                    "❌ 无法连接到动作服务器 '%s'，请执行以下步骤:", 
                    action_server_name_.c_str());
        RCLCPP_ERROR(this->get_logger(), "1. 确保Nav2已启动: ros2 action info %s", action_server_name_.c_str());
        RCLCPP_ERROR(this->get_logger(), "2. 检查动作服务器名称: ros2 action list");
        RCLCPP_ERROR(this->get_logger(), "3. 查看Nav2节点: ros2 node list");
        RCLCPP_ERROR(this->get_logger(), "4. 常见动作服务器名称:");
        RCLCPP_ERROR(this->get_logger(), "   - /red_standard_robot1/navigate_to_pose (当前配置)");
        RCLCPP_ERROR(this->get_logger(), "   - navigate_to_pose");
        RCLCPP_ERROR(this->get_logger(), "   - /navigate_to_pose");
    }
    
    // 导航到高地
    void navigateToHighGround()
    {
        if (!enabled_) {
            RCLCPP_WARN(this->get_logger(), "高地导航未启用，忽略请求");
            return;
        }
        
        // 检查服务器连接
        if (!nav_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_))) {
            RCLCPP_ERROR(this->get_logger(), 
                        "导航动作服务器未响应！当前服务器: %s", 
                        action_server_name_.c_str());
            
            // 尝试重新连接
            checkServerAvailability();
            
            // 如果重新连接失败，直接返回
            if (!nav_client_->wait_for_action_server(3s)) {
                RCLCPP_ERROR(this->get_logger(), "无法连接到导航动作服务器");
                return;
            }
        }
        
        // 创建目标点
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = high_ground_x_;
        goal_msg.pose.pose.position.y = high_ground_y_;
        goal_msg.pose.pose.position.z = high_ground_z_;
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = high_ground_orientation_w_;
        
        RCLCPP_INFO(this->get_logger(), 
                   "🚀 开始自动导航到高地坐标: (%.2f, %.2f, %.2f)", 
                   high_ground_x_, high_ground_y_, high_ground_z_);
        RCLCPP_INFO(this->get_logger(), 
                   "📡 使用的动作服务器: %s", 
                   action_server_name_.c_str());
        
        // 发送目标
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&AutoHighGroundNavigator::goalResponseCallback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&AutoHighGroundNavigator::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&AutoHighGroundNavigator::resultCallback, this, _1);
        
        try {
            auto future_goal_handle = nav_client_->async_send_goal(goal_msg, send_goal_options);
            
            // 发布状态
            auto status_msg = std_msgs::msg::Bool();
            status_msg.data = true;
            status_publisher_->publish(status_msg);
            
            RCLCPP_INFO(this->get_logger(), "🎯 导航目标已发送");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送导航目标时出错: %s", e.what());
            
            // 更新状态
            auto status_msg = std_msgs::msg::Bool();
            status_msg.data = false;
            status_publisher_->publish(status_msg);
        }
    }
    
    // 目标响应回调
    void goalResponseCallback(GoalHandleNavigate::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "❌ 高地导航目标被拒绝");
            
            auto status_msg = std_msgs::msg::Bool();
            status_msg.data = false;
            status_publisher_->publish(status_msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ 高地导航目标已接受，执行中...");
        }
    }
    
    // 反馈回调
    void feedbackCallback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        static int feedback_count = 0;
        feedback_count++;
        
        if (feedback_count % 10 == 0) {  // 每10次反馈打印一次
            RCLCPP_INFO(this->get_logger(), 
                       "📊 导航反馈: 当前位置 (%.2f, %.2f), 剩余距离: %.2f米", 
                       feedback->current_pose.pose.position.x,
                       feedback->current_pose.pose.position.y,
                       feedback->distance_remaining);
        }
    }
    
    // 结果回调
    void resultCallback(const GoalHandleNavigate::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "🎉 成功到达高地坐标！");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "❌ 高地导航被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "⏹️ 高地导航被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "❓ 高地导航未知结果");
                break;
        }
        
        // 更新状态
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = false;
        status_publisher_->publish(status_msg);
    }
    
    // 服务回调
    void startServiceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (!enabled_) {
            response->success = false;
            response->message = "高地导航未启用";
            return;
        }
        
        if (request->data) {
            RCLCPP_INFO(this->get_logger(), "收到手动启动高地导航请求");
            
            // 在单独的线程中执行导航，避免阻塞服务响应
            std::thread([this]() {
                navigateToHighGround();
            }).detach();
            
            response->success = true;
            response->message = "已开始高地导航";
        } else {
            // 取消当前目标
            if (nav_client_) {
                RCLCPP_INFO(this->get_logger(), "收到停止高地导航请求");
                // 这里可以添加取消当前目标的逻辑
            }
            response->success = true;
            response->message = "高地导航已停止";
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AutoHighGroundNavigator>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}