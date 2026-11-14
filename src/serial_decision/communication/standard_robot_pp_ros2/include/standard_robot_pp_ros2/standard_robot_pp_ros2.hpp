#ifndef STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
#define STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/sentry_posture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "std_msgs/msg/string.hpp"

namespace standard_robot_pp_ros2
{
class StandardRobotPpRos2Node : public rclcpp::Node
{
public:
  explicit StandardRobotPpRos2Node(const rclcpp::NodeOptions & options);

  ~StandardRobotPpRos2Node() override;

private:
  bool is_usb_ok_;
  bool debug_;
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  std::thread receive_thread_;
  std::thread send_thread_;
  std::thread serial_port_protect_thread_;

  rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr event_data_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr all_robot_hp_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  // Subscribe
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::SentryPosture>::SharedPtr cmd_posture_sub_;

  SendRobotCmdData send_robot_cmd_data_;
  std::mutex send_robot_cmd_data_mutex_;

  void getParams();
  void createPublisher();
  void createSubscription();
  void receiveData();
  void sendData();
  void serialPortProtect();

  void publishEventData(ReceiveEventData & data);
  void publishAllRobotHp(ReceiveAllRobotHpData & data);
  void publishGameStatus(ReceiveGameStatusData & data);
  void publishRfidStatus(ReceiveRfidStatus & data);
  void publishRobotStatus(ReceiveRobotStatus & data);

  void cmdPostureCallback(const pb_rm_interfaces::msg::SentryPosture::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  float last_hp_ = 0;
  bool has_last_hp_ = false;
};
}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
