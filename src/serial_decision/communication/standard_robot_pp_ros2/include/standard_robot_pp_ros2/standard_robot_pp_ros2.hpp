// Copyright 2025 SMBU-PolarBear-Robotics-Team
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

#ifndef STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
#define STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_state_info.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/sentry_posture.hpp"
#include "pb_rm_interfaces/msg/switch_flag.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "serial_driver/serial_driver.hpp"//在/opt/ros/humble/include/serial_driver下
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "standard_robot_pp_ros2/robot_info.hpp"

namespace standard_robot_pp_ros2
{
class StandardRobotPpRos2Node : public rclcpp::Node//类继承
{
public:
  explicit StandardRobotPpRos2Node(const rclcpp::NodeOptions & options);//构造函数以及传入参数
  //构造函数
  ~StandardRobotPpRos2Node() override;//析构函数

private:
  bool is_usb_ok_;
  bool debug_;
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;//SerialPortConfig类独占指针对象device_config_
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;//SerialDriver类独占指针对象serial_driver_
  bool record_rosbag_;
  bool set_detector_color_;

  std::thread receive_thread_;
  std::thread send_thread_;
  std::thread serial_port_protect_thread_;
  
  // Publish
  //auto pub=node->create_publisher<sensor_msgs::msg::Imu>("imu", 10);返回的类型就是rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStateInfo>::SharedPtr robot_state_info_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr event_data_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr all_robot_hp_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_motion_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr
    ground_robot_position_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::Buff>::SharedPtr buff_pub_;//收到下位机数据后进行编码成结构体后打包成ros2消息进行发布

  // Subscribe
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;//和上述同理，将上位机发送的ros2消息进行订阅写入结构体然后进行解码成字节数组发送给下位机
  rclcpp::Subscription<pb_rm_interfaces::msg::SentryPosture>::SharedPtr cmd_posture_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::SwitchFlag>::SharedPtr cmd_switch_flag_sub_;

  RobotModels robot_models_;
  std::unordered_map<std::string, rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr>
    debug_pub_map_;

  SendRobotCmdData send_robot_cmd_data_;

  void getParams();
  void createPublisher();
  void createSubscription();
  void createNewDebugPublisher(const std::string & name);
  void receiveData();
  void sendData();
  void serialPortProtect();

  void publishDebugData(ReceiveDebugData & data);
  void publishImuData(ReceiveImuData & data);
  void publishRobotInfo(ReceiveRobotInfoData & data);
  void publishEventData(ReceiveEventData & data);
  void publishAllRobotHp(ReceiveAllRobotHpData & data);
  void publishGameStatus(ReceiveGameStatusData & data);
  void publishRobotMotion(ReceiveRobotMotionData & data);
  void publishGroundRobotPosition(ReceiveGroundRobotPosition & data);
  void publishRfidStatus(ReceiveRfidStatus & data);
  void publishRobotStatus(ReceiveRobotStatus & data);
  void publishJointState(ReceiveJointState & data);
  void publishBuff(ReceiveBuff & data);

  void cmdPostureCallback(const pb_rm_interfaces::msg::SentryPosture::SharedPtr msg);
  void cmdSwitchFlagCallback(const pb_rm_interfaces::msg::SwitchFlag::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void setParam(const rclcpp::Parameter & param);
  bool getDetectColor(uint8_t robot_id, uint8_t & color);
  bool callTriggerService(const std::string & service_name);
  
  // Param client to set detect_color
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;
  std::string detector_node_name_;

  uint8_t previous_game_progress_ = 0;

  float last_hp_;
  float last_gimbal_pitch_odom_joint_, last_gimbal_yaw_odom_joint_;
};
}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_

