#include "standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"

#include <algorithm>
#include <memory>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)

using namespace std::chrono_literals;

namespace standard_robot_pp_ros2
{

StandardRobotPpRos2Node::StandardRobotPpRos2Node(const rclcpp::NodeOptions & options)
: Node("StandardRobotPpRos2Node", options),
  owned_ctx_{new IoContext(2)},            
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start StandardRobotPpRos2Node!");

  getParams();
  createPublisher();
  {
    std::lock_guard<std::mutex> lock(send_robot_cmd_data_mutex_);
    send_robot_cmd_data_ = {};
    send_robot_cmd_data_.frame_header.sof = SOF_SEND;
    send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;
    send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 6;
    send_robot_cmd_data_.time_stamp = 0;
    send_robot_cmd_data_.data.speed_vector.vx = 0;
    send_robot_cmd_data_.data.speed_vector.vy = 0;
    send_robot_cmd_data_.data.speed_vector.wz = 0;
    send_robot_cmd_data_.data.sentry_info.robot_posture = 3;
    crc8::append_CRC8_check_sum(
      reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(HeaderFrame));
  }
  createSubscription();

  serial_port_protect_thread_ = std::thread(&StandardRobotPpRos2Node::serialPortProtect, this);
  receive_thread_ = std::thread(&StandardRobotPpRos2Node::receiveData, this);
  send_thread_ = std::thread(&StandardRobotPpRos2Node::sendData, this);
}

StandardRobotPpRos2Node::~StandardRobotPpRos2Node()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_port_protect_thread_.joinable()) {
    serial_port_protect_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void StandardRobotPpRos2Node::createPublisher()
{
  event_data_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
  all_robot_hp_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
  game_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
}

void StandardRobotPpRos2Node::createSubscription()
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&StandardRobotPpRos2Node::cmdVelCallback, this, std::placeholders::_1));
  cmd_posture_sub_ = this->create_subscription<pb_rm_interfaces::msg::SentryPosture>(
    "/cmd/sentry_posture", 10,std::bind(&StandardRobotPpRos2Node::cmdPostureCallback, this,std::placeholders::_1));  //订阅ros消息然后进行解码处理通过串口发送给下位机
}

void StandardRobotPpRos2Node::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};  
  auto fc = FlowControl::NONE;  
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  } 

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");
    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");
    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");
    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
    baud_rate, fc, pt,
    sb);
}

/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::
  serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  // @TODO: 1.保持串口连接 2.串口断开重连 3.串口异常处理

  // 初始化串口
  serial_driver_->init_port(device_name_, *device_config_);
  // 尝试打开串口
  try {
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Serial port opened!");
      is_usb_ok_ = true;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
    is_usb_ok_ = false;
  }  //初次尝试打开串口

  // is_usb_ok_ = true;
  std::this_thread::sleep_for(
    std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME)); 

  while (rclcpp::ok())  
  {
    if (!is_usb_ok_)  
    {
      try {
        if (serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }  

        serial_driver_->port()->open();  

        if (serial_driver_->port()->is_open()) {
          RCLCPP_INFO(get_logger(), "Serial port opened!");
          is_usb_ok_ = true;  
        }
        
      } catch (const std::exception & ex) {
        is_usb_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
      }
    }

   
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME)); 
  }  
}

/********************************************************/
/* Receive data                                         */
/********************************************************/



void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");

  //匿名函数，用于接收指定长度的数据
  auto receive_exact = [this](uint8_t * data, size_t length) {
    size_t received = 0;
    while (received < length) {
      std::vector<uint8_t> chunk(length - received);
      const size_t count = serial_driver_->port()->receive(chunk);
      if (count == 0 || count > chunk.size()) {
        return false;
      }
      std::copy_n(chunk.begin(), count, data + received);
      received += count;
    }
    return true;
  };

  std::vector<uint8_t> sof(1);

  int sof_count = 0;
  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "receive: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      if (!receive_exact(sof.data(), sof.size())) {
        continue;
      }
      if (sof[0] != SOF_RECEIVE) {
        sof_count++;
        RCLCPP_INFO(get_logger(), "Find sof, cnt=%d", sof_count);
        continue;
      }

      // Reset sof_count when SOF_RECEIVE is found
      sof_count = 0;  

      // sof[0] == SOF_RECEIVE 后读取剩余 header_frame 内容
      std::vector<uint8_t> header_frame_buf(sizeof(HeaderFrame));
      header_frame_buf[0] = sof[0];
      if (!receive_exact(header_frame_buf.data() + 1, sizeof(HeaderFrame) - 1)) {
        continue;
      }
      HeaderFrame header_frame =
        fromVector<HeaderFrame>(header_frame_buf);

      // HeaderFrame CRC8 check
      bool crc8_ok = crc8::verify_CRC8_check_sum(
        reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
      if (!crc8_ok) {
        RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
        continue;
      }

      const size_t frame_size = sizeof(HeaderFrame) + header_frame.len + sizeof(uint16_t);
      if (frame_size > 64) {
          RCLCPP_WARN(
            get_logger(), "Discarding oversized frame: %zu bytes", frame_size);
          continue;
      }

      std::vector<uint8_t> data_buf(frame_size);
      std::copy(header_frame_buf.begin(), header_frame_buf.end(), data_buf.begin());
      if (!receive_exact(
          data_buf.data() + sizeof(HeaderFrame), header_frame.len + sizeof(uint16_t)))
      {
        continue;
      }

      size_t expected_frame_size = 0;
      switch (header_frame.id) {
        case ID_EVENT_DATA:
          expected_frame_size = sizeof(ReceiveEventData);
          break;
        case ID_ALL_ROBOT_HP:
          expected_frame_size = sizeof(ReceiveAllRobotHpData);
          break;
        case ID_GAME_STATUS:
          expected_frame_size = sizeof(ReceiveGameStatusData);
          break;
        case ID_RFID_STATUS:
          expected_frame_size = sizeof(ReceiveRfidStatus);
          break;
        case ID_ROBOT_STATUS:
          expected_frame_size = sizeof(ReceiveRobotStatus);
          break;
        default:
          RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
          continue;
      }
      if (data_buf.size() != expected_frame_size) {
        RCLCPP_WARN(
          get_logger(), "Discarding frame with unexpected length: id=%d, size=%zu",
          header_frame.id, data_buf.size());
        continue;
      }

      // 整包数据校验
      bool crc16_ok = crc16::verify_CRC16_check_sum(data_buf);
      if (!crc16_ok) {
        RCLCPP_ERROR(get_logger(), "Data segment CRC16 error!");
        continue;
      }

      switch (header_frame.id)
      {
        case ID_EVENT_DATA: {
          ReceiveEventData event_data = fromVector<ReceiveEventData>(data_buf);
          publishEventData(event_data);
        } break;
        case ID_ALL_ROBOT_HP: {
          ReceiveAllRobotHpData all_robot_hp_data = fromVector<ReceiveAllRobotHpData>(data_buf);
          publishAllRobotHp(all_robot_hp_data);
        } break;
        case ID_GAME_STATUS: {
          ReceiveGameStatusData game_status_data = fromVector<ReceiveGameStatusData>(data_buf);
          publishGameStatus(game_status_data);
        } break;
        case ID_RFID_STATUS: {
          ReceiveRfidStatus rfid_status_data = fromVector<ReceiveRfidStatus>(data_buf);
          publishRfidStatus(rfid_status_data);
        } break;
        case ID_ROBOT_STATUS: {
          ReceiveRobotStatus robot_status_data = fromVector<ReceiveRobotStatus>(data_buf);
          publishRobotStatus(robot_status_data);
        } break;
        default: break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;
    }
  }
}

void StandardRobotPpRos2Node::publishEventData(ReceiveEventData & event_data)
{
  pb_rm_interfaces::msg::EventData msg;

  msg.supply_zone = event_data.data.supply_zone;

  msg.small_energy = event_data.data.small_energy;
  msg.big_energy = event_data.data.big_energy;

  msg.r_outpost = event_data.data.R_outpost;
  msg.b_outpost = event_data.data.B_outpost;

  msg.r_outpost_control = event_data.data.R_outpost_control;
  msg.b_outpost_control = event_data.data.B_outpost_control;

  event_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishAllRobotHp(ReceiveAllRobotHpData & all_robot_hp)
{
  pb_rm_interfaces::msg::GameRobotHP msg;

  msg.ally_1_robot_hp = all_robot_hp.data.ally_1_robot_HP;
  msg.ally_3_robot_hp = all_robot_hp.data.ally_3_robot_HP;
  msg.ally_4_robot_hp = all_robot_hp.data.ally_4_robot_HP;
  msg.ally_7_robot_hp = all_robot_hp.data.ally_7_robot_HP;
  msg.outpost_r_hp = all_robot_hp.data.outpost_R_HP;
  msg.outpost_b_hp = all_robot_hp.data.outpost_B_HP;
  msg.ally_base_hp = all_robot_hp.data.ally_base_HP;
  msg.enemy_base_hp = all_robot_hp.data.enemy_base_HP;
  msg.ally_control_degree = all_robot_hp.data.ally_control_degree;
  msg.enemy_control_degree = all_robot_hp.data.enemy_control_degree;

  all_robot_hp_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishGameStatus(ReceiveGameStatusData & game_status)
{
  pb_rm_interfaces::msg::GameStatus msg;

  msg.game_progress = game_status.data.game_progress;
  msg.stage_remain_time = game_status.data.stage_remain_time;

  game_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRfidStatus(ReceiveRfidStatus & rfid_status)
{
  pb_rm_interfaces::msg::RfidStatus msg;

  msg.ally_supply_point = rfid_status.data.ally_supply_point;
  msg.r_outpost_point = rfid_status.data.R_outpost_point;
  msg.b_outpost_point = rfid_status.data.B_outpost_point;

  rfid_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRobotStatus(ReceiveRobotStatus & robot_status)
{
  pb_rm_interfaces::msg::RobotStatus msg;

  msg.current_hp = robot_status.data.current_up;
  msg.maximum_hp = robot_status.data.maximum_hp;
  msg.projectile_allowance_17mm = robot_status.data.projectile_allowance_17mm;
  msg.remaining_gold_coin = robot_status.data.remaining_gold_coin;
  msg.powerful_offensive_posture = robot_status.data.powerful_offensive_posture;
  msg.powerful_defensive_posture = robot_status.data.powerful_defensive_posture;
  msg.powerful_movement_posture = robot_status.data.powerful_movement_posture;

  if (has_last_hp_ && last_hp_ - msg.current_hp > 0) {
    msg.is_hp_deduced = true;
  }
  last_hp_ = robot_status.data.current_up;  
  has_last_hp_ = true;

  robot_status_pub_->publish(msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData!");

  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      SendRobotCmdData packet_snapshot;
      {
        std::lock_guard<std::mutex> lock(send_robot_cmd_data_mutex_);
        packet_snapshot = send_robot_cmd_data_;
      }
      crc16::append_CRC16_check_sum(
        reinterpret_cast<uint8_t *>(&packet_snapshot), sizeof(SendRobotCmdData));
      std::vector<uint8_t> send_robot_cmd_data = toVector(packet_snapshot);
      serial_driver_->port()->send(send_robot_cmd_data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      is_usb_ok_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void StandardRobotPpRos2Node::cmdPostureCallback(
  const pb_rm_interfaces::msg::SentryPosture::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(send_robot_cmd_data_mutex_);
  send_robot_cmd_data_.data.sentry_info.robot_posture = msg->current_posture;
}

void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(send_robot_cmd_data_mutex_);
  send_robot_cmd_data_.data.speed_vector.vx = msg->linear.x;
  send_robot_cmd_data_.data.speed_vector.vy = msg->linear.y;
  send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z;
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::StandardRobotPpRos2Node)
