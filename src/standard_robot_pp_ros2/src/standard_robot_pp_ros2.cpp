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

//完整流程：三个阻塞多线程：（1）保护串口线程；（2）发送数据线程；（3）收到数据线程
//收发逻辑：收数据——从下位机收取到字节流（std::vector<uint8_t>的字节数组）然后转成ros2自定义消息类型的消息去发布话题
//发数据：一样是订阅话题然后把ros2自定义消息类型的消息去转成结构体,将结构体数据再变成字节数组进行发送给下位机

#include "standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"

#include <memory>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)

using namespace std::chrono_literals;

namespace standard_robot_pp_ros2
{

StandardRobotPpRos2Node::StandardRobotPpRos2Node(const rclcpp::NodeOptions & options)
: Node("StandardRobotPpRos2Node", options),//父类构造函数
  owned_ctx_{new IoContext(2)},//owned_ctx=new IoContext(2)
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}//serial_driver=new drivers::serial_driver::SerialDriver(*owned_ctx_)
  //把IO上下文传递给串口驱动程序
  //RAII（Resource Acquisition Is Initialization）的完整闭环——资源获取即初始化，生命周期结束即释放
  //构造函数初始化列表
{
  RCLCPP_INFO(get_logger(), "Start StandardRobotPpRos2Node!");

  getParams();
  createPublisher();
  createSubscription();

  robot_models_.chassis = {
    {0, "无底盘"}, {1, "麦轮底盘"}, {2, "全向轮底盘"}, {3, "舵轮底盘"}, {4, "平衡底盘"}};
  robot_models_.gimbal = {{0, "无云台"}, {1, "yaw_pitch直连云台"}};
  robot_models_.shoot = {{0, "无发射机构"}, {1, "摩擦轮+拨弹盘"}, {2, "气动+拨弹盘"}};
  robot_models_.arm = {{0, "无机械臂"}, {1, "mini机械臂"}};
  robot_models_.custom_controller = {{0, "无自定义控制器"}, {1, "mini自定义控制器"}};//一样是串口去确定机器人类型和一些参数
  
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
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/imu", 10);//利用这个节点的对象的模板成员函数（创建发布者函数）去发布一个话题，传入参数是话题名称和qos队列服务质量，返回的是指向这个模板成员函数类型的发布者对象的共享指针
  robot_state_info_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStateInfo>("serial/robot_state_info", 10);
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
  robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("serial/robot_motion", 10);
  event_data_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
  all_robot_hp_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
  game_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  ground_robot_position_pub_ = this->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", 10);
  rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  buff_pub_ = this->create_publisher<pb_rm_interfaces::msg::Buff>("referee/buff", 10);//创建ros消息发布者：对从下位机接收的字节流进行编码然后打包成ros消息然后进行发布
}

void StandardRobotPpRos2Node::createNewDebugPublisher(const std::string & name)
{
  RCLCPP_INFO(get_logger(), "Create new debug publisher: %s", name.c_str());
  std::string topic_name = "serial/debug/" + name;
  auto debug_pub = this->create_publisher<example_interfaces::msg::Float64>(topic_name, 10);
  debug_pub_map_.insert(std::make_pair(name, debug_pub));//不懂先跳过
}

void StandardRobotPpRos2Node::createSubscription()
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&StandardRobotPpRos2Node::cmdVelCallback, this, std::placeholders::_1));

  // cmd_gimbal_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //   "cmd_gimbal_joint", 10,
  //   std::bind(&StandardRobotPpRos2Node::cmdGimbalJointCallback, this, std::placeholders::_1));//自瞄配合gimbal_manager一起看

  // cmd_shoot_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
  //   "cmd_shoot", 10,
  //   std::bind(&StandardRobotPpRos2Node::cmdShootCallback, this, std::placeholders::_1));//自瞄
  // cmd_tracking_sub_ = this->create_subscription<std_msgs::msg::String>(
  //       "auto_aim_target_pos", 10,
  //       std::bind(&StandardRobotPpRos2Node::trackingCallback, this, std::placeholders::_1));
  cmd_posture_sub_ = this->create_subscription<pb_rm_interfaces::msg::SentryPosture>(
        "/cmd/sentry_posture", 
        10, 
        std::bind(&StandardRobotPpRos2Node::cmdPostureCallback, this, std::placeholders::_1));//订阅ros消息然后进行解码处理通过串口发送给下位机
  cmd_switch_flag_sub_ = this->create_subscription<pb_rm_interfaces::msg::SwitchFlag>(
        "/cmd/switch_flag",
        10,
        std::bind(&StandardRobotPpRos2Node::cmdSwitchFlagCallback,this,std::placeholders::_1));
}


void StandardRobotPpRos2Node::getParams()//串口驱动的参数加载函数，从ros2参数服务器读取配置并校验
//整体逻辑:yaml配置文件->declare_parameter()读取->校验合法性（非法直接抛出异常）->填入SerialPortConfig
{
  using FlowControl = drivers::serial_driver::FlowControl;
  //cpp类型别名声明（类似与引用——起别名）
  //引用起别名：int a=10;int &b=a;在这里就是之间把a变量起了一个别名b
  //等价于typedef drivers::serial_driver::FlowControl FlowControl
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};//c++11值初始化语法，用空大括号初始化{}，这个初始化是将此变量变为0
  auto fc = FlowControl::NONE;//fc是drivers::serial_driver::FlowControl
  //fc这个变量的类型就是drivers::serial_driver::FlowControl，具体的值就是在这个枚举类型下的NONE
  //同时带上FlowControl::不会污染外层命名空间，如果只用普通枚举类型，会导致编译器无法处理到底是FlowControl::NONE还是Parity::NONE
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;
  //对于初始化变量的好处就是即使后续参数读取或者写入失败，这些参数也有明确的初始状态，不会出现未定义行为

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");//给成员变量赋值
    //declare_parameter<T>(name, default) ：声明这个参数，必须最先调用，向参数服务器注册这个参数并设置默认值，返回参数当前的值，一个参数只需要declare一次
    //get_parameter<T>(name,variable):前提是必须声明或者说创建过这个变量，不修改默认值，不注册任何东西，只是进行读取，将name的值读取到传入variable
    //device_name: /dev/ttyACM0
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }//异常处理：对于参数类型获取异常的处理——这里就是对于字符串类型的"device_name"获取的异常处理
  //如果在launch文件中进行对节点参数列表的重新载入，由于yaml文件解释器会自己解析相关参数类型，只有和指定即注册过的参数类型对上才不会抛出异常

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }//波特率变量定义publishGameStatus
   //baud_rate: 115200

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");
    //只读常量，即定义初始化后不可以被修改
    //parity: none
    if (fc_string == "none") {
      fc = FlowControl::NONE;//
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }//抛出空异常
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");
    //parity: none
    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }//空参数抛出异常
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;//参数类型错误抛出异常
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");
    //stop_bits: "1"
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

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);//device_config_独占指针指向SerialPortConfig类对象，传入相关实际参数进行构造函数初始化和相关成员变量初始化
    //独占指针是不可拷贝的，只能std::move转移所有权
  record_rosbag_ = declare_parameter("record_rosbag", false);
  set_detector_color_ = declare_parameter("set_detector_color", false);
  debug_ = declare_parameter("debug", false);
}


/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::serialPortProtect()//这个函数是直接注册绑定在serial_port_protect_thread_这个线程中的
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  // @TODO: 1.保持串口连接 2.串口断开重连 3.串口异常处理

  // 初始化串口
  serial_driver_->init_port(device_name_, *device_config_);
  //以后所有的数据收发就是通过这个串口指针对象
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
  }//初次尝试打开串口

  is_usb_ok_ = true;//具体对于覆盖操作的理解？
  //ai解释：判断usb是否正常的责任，从循环守护转移到数据收到中进行判断，尝试初次打开后直接把usb的状态置成正常后，后续在读写操作中对usb状态进行判断，（若出现问题进行循环守护）
  //整体逻辑上：其实对于usb状态就是一个对于串口是否打开的标志位
  std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));//等待串口线程的稳定

  while (rclcpp::ok())//只要ros2仍然在运行
   {
    if (!is_usb_ok_)//如果usb断开（目前看下来usb断开的情况只有在发现数据传输过程中） 
    {
      try {
        if (serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }//在usb断开的情况下，如果之前串口是开着的，先关掉

        serial_driver_->port()->open();//尝试进行重新打开

        if (serial_driver_->port()->is_open()) {
          RCLCPP_INFO(get_logger(), "Serial port opened!");
          is_usb_ok_ = true;//如果成功重新打开串口，就将usb状态变成正常打开
        }
        //对于在cpp中，这里的if是直接顺序执行的，并不是if-else或者switch-case这种互斥关系，语句块的执行一般按照顺序进行执行操作
      } catch (const std::exception & ex) {
        is_usb_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());//捕捉异常并进行抛出
      }
    }

    // thread sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));//每1s循环一次
  }//循环守护串口线程
}

/********************************************************/
/* Receive data                                         */
/********************************************************/

//收到数据的主逻辑

void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");

  std::vector<uint8_t> sof(1);//大小为一个uint8元素大小的字节数组作为缓冲区
  std::vector<uint8_t> receive_data;

  int sof_count = 0;
  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "receive: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      serial_driver_->port()->receive(sof);
      //receive函数：
      //Bocking receive operation        ← 阻塞操作
      //A buffer to be populated with the read data（传入填充读取数据的缓冲区，这里就是sof(1)）
      //The number of bytes read（返回值——实际读取到的字节数）
      //receive(buffer)
      //读取字节数：由buffer.size()
      //读取的数据放在buffer本身（覆盖写入——函数本身执行的核心操作逻辑）
      //返回值：实际读取到字节数
      //现在就是从初始化之后的串口中读取数据——只读取头帧的sof写入到std::vector<uint8_t> sof(1)这个临时缓冲区中

      //对于收到的帧头不是指定要收到的就一直阻塞
      if (sof[0] != SOF_RECEIVE) {
        sof_count++;
        RCLCPP_INFO(get_logger(), "Find sof, cnt=%d", sof_count);
        continue;
      }

      // Reset sof_count when SOF_RECEIVE is found
      sof_count = 0;//如果收到是的帧头sof是0x05A就重置sof_count

      // sof[0] == SOF_RECEIVE 后读取剩余 header_frame 内容
      // 确认sof[0] ==  SOF_RECEIVE之后开始读取头帧剩余数据
      std::vector<uint8_t> header_frame_buf(3);  // sof 在读取完数据后添加
      //头帧的数据端是sof len id crc,读取完0x05A数据段之后接着读取剩下数据
      serial_driver_->port()->receive(header_frame_buf);  // 读取除 sof 外剩下的头包数据
      header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加 sof
      //在数组容器第一个位置上插入sof数据打包成完整的帧头包
      HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);//反序列化成cpp定义的结构体数据

      // HeaderFrame CRC8 check
      bool crc8_ok = crc8::verify_CRC8_check_sum(
        reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
      if (!crc8_ok) {
        RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
        continue;
      }
      //对收到的头帧包进行crc8校验
      //如果crc8失败之后就直接跳过本次循环重新读取头帧开始一遍上述流程

      // crc8_ok 校验正确后读取数据段
      // 根据数据段长度读取数据
      std::vector<uint8_t> data_buf(header_frame.len + 2);  // len + crc(time_stamp + data + crc)
      //下位机发送的头帧数据的中的len是对于后续数据段的长度的发送（由于是uint8数据类型所以发送数据段len在0到255之间，换言之就是data有0到255字节的大小）
      //  帧结构（len=time_stamp+data,data前有一个time_stamp）:
      // ┌──────┬──────┬──────┬──────┬───────────┬────────┐
      // │ SOF  │ len  │  id  │ crc8 │   data    │ crc16  │
      // │ 1B   │ 1B   │ 1B   │ 1B   │  len 字节  │  2B    │
      // └──────┴──────┴──────┴──────┴────────────┴────────┘
      //          ↑
      //    这个 len 就是 header_frame.len
      //    它告诉接收方：后面还有len字节大小的数据要读

      int received_len = serial_driver_->port()->receive(data_buf);//第一次收取到数据段+crc16
      int received_len_sum = received_len;//初始化收到的总字节大小是第一次收取的数据段+crc16
      // 考虑到一次性读取数据可能存在数据量过大，读取不完整的情况。需要检测是否读取完整
      // 计算剩余未读取的数据长度
      int remain_len = header_frame.len + 2 - received_len;//所有剩余的数据段
      while (remain_len > 0) {  // 读取剩余未读取的数据
        std::vector<uint8_t> remain_buf(remain_len);
        received_len = serial_driver_->port()->receive(remain_buf);//把剩余全部的数据段进行读取然后维护内部的received_len变量
        data_buf.insert(data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
        //如果数据太大而导致数据没有完全接收，就进行检验之后重新对剩余的数据进行收取，然后重新组合成完整的收到的数据帧
        received_len_sum += received_len;//在int received_len_sum = received_len初始化收到的总字节大小是第一次收取的数据段+crc16基础上进行收到数据的类加，不断写入应该收到的完整数据中
        remain_len -= received_len;//对于剩余的数据进行不断累减
      }//直到剩余的数据全部减成0

      // 数据段读取完成后添加 header_frame_buf 到 data_buf，得到完整数据包
      data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

      //这个就是debug是true时不会跳过本次循环，接着进行debug模式下的crc16整包校验
      if (!debug_ && header_frame.id == ID_DEBUG) {
        continue;
      }//如果没有开debug测试模式以及头包id是debug的id才会跳过本次循环进入下一次循环
      //debug是false且头包是debug的id的时候就跳过这次循环，说白了就是如果有一次循环是debug数据包就不处理这个debug数据包（不进行crc16校验）
      //debug是false且头包id是其他id的时候不跳过本次循环接着进行数据包校验处理
      

      // 整包数据校验
      bool crc16_ok = crc16::verify_CRC16_check_sum(data_buf);
      if (!crc16_ok) {
        RCLCPP_ERROR(get_logger(), "Data segment CRC16 error!");
        continue;
      }//crc校验后续进行学习

      // crc16_ok 校验正确后根据 header_frame.id 解析数据
      switch (header_frame.id) //选择头包id进行数据解析（这一块就是进行解包然后打包成ros消息进行发布的环节）
      {
        case ID_DEBUG: {
          ReceiveDebugData debug_data = fromVector<ReceiveDebugData>(data_buf);//把字节反序列化成结构体
          publishDebugData(debug_data);
        } break;
        case ID_IMU: {
          ReceiveImuData imu_data = fromVector<ReceiveImuData>(data_buf);
          publishImuData(imu_data);
        } break;
        case ID_ROBOT_STATE_INFO: {
          ReceiveRobotInfoData robot_info_data = fromVector<ReceiveRobotInfoData>(data_buf);
          publishRobotInfo(robot_info_data);
        } break;
        case ID_EVENT_DATA: {
          ReceiveEventData event_data = fromVector<ReceiveEventData>(data_buf);
          publishEventData(event_data);
        } break;
        case ID_PID_DEBUG: {
          RCLCPP_WARN(get_logger(), "Not implemented yet!");
        } break;
        case ID_ALL_ROBOT_HP: {
          ReceiveAllRobotHpData all_robot_hp_data = fromVector<ReceiveAllRobotHpData>(data_buf);
          publishAllRobotHp(all_robot_hp_data);
        } break;
        case ID_GAME_STATUS: {
          ReceiveGameStatusData game_status_data = fromVector<ReceiveGameStatusData>(data_buf);
          publishGameStatus(game_status_data);
        } break;
        case ID_ROBOT_MOTION: {
          ReceiveRobotMotionData robot_motion_data = fromVector<ReceiveRobotMotionData>(data_buf);
          publishRobotMotion(robot_motion_data);
        } break;
        case ID_GROUND_ROBOT_POSITION: {
          ReceiveGroundRobotPosition ground_robot_position_data =
            fromVector<ReceiveGroundRobotPosition>(data_buf);
          publishGroundRobotPosition(ground_robot_position_data);
        } break;
        case ID_RFID_STATUS: {
          ReceiveRfidStatus rfid_status_data = fromVector<ReceiveRfidStatus>(data_buf);
          publishRfidStatus(rfid_status_data);
        } break;
        case ID_ROBOT_STATUS: {
          ReceiveRobotStatus robot_status_data = fromVector<ReceiveRobotStatus>(data_buf);
          publishRobotStatus(robot_status_data);
        } break;
        case ID_JOINT_STATE: {
          ReceiveJointState joint_state_data = fromVector<ReceiveJointState>(data_buf);
          publishJointState(joint_state_data);
        } break;
        case ID_BUFF: {
          ReceiveBuff buff = fromVector<ReceiveBuff>(data_buf);
          publishBuff(buff);
        } break;
        default: {
          RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
        } break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;
    }
  }
}

void StandardRobotPpRos2Node::publishDebugData(ReceiveDebugData & received_debug_data)
{
  static rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr debug_pub;
  for (auto & package : received_debug_data.packages) {
    // Create a vector to hold the non-zero data
    std::vector<uint8_t> non_zero_data;
    for (unsigned char name : package.name) {
      if (name != 0) {
        non_zero_data.push_back(name);
      } else {
        break;
      }
    }
    // Convert the non-zero data to a string
    std::string name(non_zero_data.begin(), non_zero_data.end());

    if (name.empty()) {
      continue;
    }

    if (debug_pub_map_.find(name) == debug_pub_map_.end()) {
      createNewDebugPublisher(name);
    }
    debug_pub = debug_pub_map_.at(name);

    example_interfaces::msg::Float64 msg;
    msg.data = package.data;
    debug_pub->publish(msg);
  }
}

void StandardRobotPpRos2Node::publishImuData(ReceiveImuData & imu_data)
{
  sensor_msgs::msg::JointState joint_msg;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = joint_msg.header.stamp = now();
  imu_msg.header.frame_id = "gimbal_pitch_odom";

  // Convert Euler angles to quaternion
  tf2::Quaternion q;
  q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);//setRPY是tf2::Quaternion的成员函数，是把欧拉角转成四元数存入q这个对象中
  imu_msg.orientation = tf2::toMsg(q);//转成geometry_msgs/Quaternion orientation这个ros2的消息类型
  //四元数表示整个车的姿态
  imu_msg.angular_velocity.x = imu_data.data.roll_vel;
  imu_msg.angular_velocity.y = imu_data.data.pitch_vel;
  imu_msg.angular_velocity.z = imu_data.data.yaw_vel;
  //刚体三个轴的角速度
  imu_pub_->publish(imu_msg);//imu_pub_是<sensor_msgs::msg::Imu>类型类的共享指针对象，用这个实例化对象去调用成员函数去发布消息
  //这个只是纯发布imu的相关数据

  joint_msg.name = {
    "gimbal_pitch_joint",
    "gimbal_yaw_joint",
    "gimbal_pitch_odom_joint",
    "gimbal_yaw_odom_joint",
  };
  joint_msg.position = {
    imu_data.data.pitch,
    imu_data.data.yaw,
    last_gimbal_pitch_odom_joint_,
    last_gimbal_yaw_odom_joint_,
  };
  joint_state_pub_->publish(joint_msg);//这个主要是参与tf树的构建
}

void StandardRobotPpRos2Node::publishRobotInfo(ReceiveRobotInfoData & robot_info)
{
  pb_rm_interfaces::msg::RobotStateInfo msg;

  msg.header.stamp.sec = robot_info.time_stamp / 1000;
  msg.header.stamp.nanosec = (robot_info.time_stamp % 1000) * 1e6;
  msg.header.frame_id = "odom";

  msg.models.chassis = robot_models_.chassis.at(robot_info.data.type.chassis);
  msg.models.gimbal = robot_models_.gimbal.at(robot_info.data.type.gimbal);
  msg.models.shoot = robot_models_.shoot.at(robot_info.data.type.shoot);
  msg.models.arm = robot_models_.arm.at(robot_info.data.type.arm);
  msg.models.custom_controller =
    robot_models_.custom_controller.at(robot_info.data.type.custom_controller);

  robot_state_info_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishEventData(ReceiveEventData & event_data)
{
  pb_rm_interfaces::msg::EventData msg;

  msg.non_overlapping_supply_zone = event_data.data.non_overlapping_supply_zone;
  msg.overlapping_supply_zone = event_data.data.overlapping_supply_zone;
  msg.supply_zone = event_data.data.supply_zone;

  msg.small_energy = event_data.data.small_energy;
  msg.big_energy = event_data.data.big_energy;

  msg.central_highland = event_data.data.central_highland;
  msg.trapezoidal_highland = event_data.data.trapezoidal_highland;

  msg.center_gain_zone = event_data.data.center_gain_zone;

  event_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishAllRobotHp(ReceiveAllRobotHpData & all_robot_hp)
{
  pb_rm_interfaces::msg::GameRobotHP msg;

  // msg.red_1_robot_hp = all_robot_hp.data.red_1_robot_hp;
  // msg.red_2_robot_hp = all_robot_hp.data.red_2_robot_hp;
  // msg.red_3_robot_hp = all_robot_hp.data.red_3_robot_hp;
  // msg.red_4_robot_hp = all_robot_hp.data.red_4_robot_hp;
  // msg.red_7_robot_hp = all_robot_hp.data.red_7_robot_hp;
  // msg.red_outpost_hp = all_robot_hp.data.red_outpost_hp;
  // msg.red_base_hp = all_robot_hp.data.red_base_hp;

  // msg.blue_1_robot_hp = all_robot_hp.data.blue_1_robot_hp;
  // msg.blue_2_robot_hp = all_robot_hp.data.blue_2_robot_hp;
  // msg.blue_3_robot_hp = all_robot_hp.data.blue_3_robot_hp;
  // msg.blue_4_robot_hp = all_robot_hp.data.blue_4_robot_hp;
  // msg.blue_7_robot_hp = all_robot_hp.data.blue_7_robot_hp;
  // msg.blue_outpost_hp = all_robot_hp.data.blue_outpost_hp;
  // msg.blue_base_hp = all_robot_hp.data.blue_base_hp;

  msg.ally_1_robot_hp = all_robot_hp.data.ally_1_robot_HP;
  msg.ally_2_robot_hp = all_robot_hp.data.ally_2_robot_HP;
  msg.ally_3_robot_hp = all_robot_hp.data.ally_3_robot_HP;
  msg.ally_4_robot_hp = all_robot_hp.data.ally_4_robot_HP;
  msg.ally_7_robot_hp = all_robot_hp.data.ally_7_robot_HP;
  msg.ally_outpost_hp = all_robot_hp.data.ally_outpost_HP;
  msg.ally_base_hp = all_robot_hp.data.ally_base_HP;
  msg.enemy_outpost_flag=all_robot_hp.data.enemy_outpost_flag;

  all_robot_hp_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishGameStatus(ReceiveGameStatusData & game_status)
{
  pb_rm_interfaces::msg::GameStatus msg;
  msg.game_progress = game_status.data.game_progress;
  msg.stage_remain_time = game_status.data.stage_remain_time;
  game_status_pub_->publish(msg);

  //开rosbag时根据比赛阶段去进行记录比赛数据
  if (record_rosbag_ && game_status.data.game_progress != previous_game_progress_) {
    previous_game_progress_ = game_status.data.game_progress;
    RCLCPP_INFO(get_logger(), "Game progress: %d", game_status.data.game_progress);

    std::string service_name;
    switch (game_status.data.game_progress) {
      case pb_rm_interfaces::msg::GameStatus::COUNT_DOWN:
        service_name = "start_recording";
        break;
      case pb_rm_interfaces::msg::GameStatus::GAME_OVER:
        service_name = "stop_recording";
        break;
      default:
        return;
    }

    if (!callTriggerService(service_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to call service: %s", service_name.c_str());
    }
  }
}

void StandardRobotPpRos2Node::publishRobotMotion(ReceiveRobotMotionData & robot_motion)
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = robot_motion.data.speed_vector.vx;
  msg.linear.y = robot_motion.data.speed_vector.vy;
  msg.angular.z = robot_motion.data.speed_vector.wz;

  robot_motion_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishGroundRobotPosition(
  ReceiveGroundRobotPosition & ground_robot_position)
{
  pb_rm_interfaces::msg::GroundRobotPosition msg;

  msg.hero_position.x = ground_robot_position.data.hero_x;
  msg.hero_position.y = ground_robot_position.data.hero_y;

  msg.engineer_position.x = ground_robot_position.data.engineer_x;
  msg.engineer_position.y = ground_robot_position.data.engineer_y;

  msg.standard_3_position.x = ground_robot_position.data.standard_3_x;
  msg.standard_3_position.y = ground_robot_position.data.standard_3_y;

  msg.standard_4_position.x = ground_robot_position.data.standard_4_x;
  msg.standard_4_position.y = ground_robot_position.data.standard_4_y;

  ground_robot_position_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRfidStatus(ReceiveRfidStatus & rfid_status)
{
  pb_rm_interfaces::msg::RfidStatus msg;

  msg.base_gain_point = rfid_status.data.base_gain_point;
  msg.central_highland_gain_point = rfid_status.data.central_highland_gain_point;
  msg.enemy_central_highland_gain_point = rfid_status.data.enemy_central_highland_gain_point;
  msg.friendly_trapezoidal_highland_gain_point =
    rfid_status.data.friendly_trapezoidal_highland_gain_point;
  msg.enemy_trapezoidal_highland_gain_point =
    rfid_status.data.enemy_trapezoidal_highland_gain_point;
  msg.friendly_fly_ramp_front_gain_point = rfid_status.data.friendly_fly_ramp_front_gain_point;
  msg.friendly_fly_ramp_back_gain_point = rfid_status.data.friendly_fly_ramp_back_gain_point;
  msg.enemy_fly_ramp_front_gain_point = rfid_status.data.enemy_fly_ramp_front_gain_point;
  msg.enemy_fly_ramp_back_gain_point = rfid_status.data.enemy_fly_ramp_back_gain_point;
  msg.friendly_central_highland_lower_gain_point =
    rfid_status.data.friendly_central_highland_lower_gain_point;
  msg.friendly_central_highland_upper_gain_point =
    rfid_status.data.friendly_central_highland_upper_gain_point;
  msg.enemy_central_highland_lower_gain_point =
    rfid_status.data.enemy_central_highland_lower_gain_point;
  msg.enemy_central_highland_upper_gain_point =
    rfid_status.data.enemy_central_highland_upper_gain_point;
  msg.friendly_highway_lower_gain_point = rfid_status.data.friendly_highway_lower_gain_point;
  msg.friendly_highway_upper_gain_point = rfid_status.data.friendly_highway_upper_gain_point;
  msg.enemy_highway_lower_gain_point = rfid_status.data.enemy_highway_lower_gain_point;
  msg.enemy_highway_upper_gain_point = rfid_status.data.enemy_highway_upper_gain_point;
  msg.friendly_fortress_gain_point = rfid_status.data.friendly_fortress_gain_point;
  msg.friendly_outpost_gain_point = rfid_status.data.friendly_outpost_gain_point;
  msg.friendly_supply_zone_non_exchange = rfid_status.data.friendly_supply_zone_non_exchange;
  msg.friendly_supply_zone_exchange = rfid_status.data.friendly_supply_zone_exchange;
  msg.friendly_big_resource_island = rfid_status.data.friendly_big_resource_island;
  msg.enemy_big_resource_island = rfid_status.data.enemy_big_resource_island;
  msg.center_gain_point = rfid_status.data.center_gain_point;

  rfid_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRobotStatus(ReceiveRobotStatus & robot_status)
{
  pb_rm_interfaces::msg::RobotStatus msg;

  msg.robot_id = robot_status.data.robot_id;
  msg.robot_level = robot_status.data.robot_level;
  msg.current_hp = robot_status.data.current_up;
  msg.maximum_hp = robot_status.data.maximum_hp;
  msg.shooter_barrel_cooling_value = robot_status.data.shooter_barrel_cooling_value;
  msg.shooter_barrel_heat_limit = robot_status.data.shooter_barrel_heat_limit;
  msg.shooter_17mm_1_barrel_heat = robot_status.data.shooter_17mm_1_barrel_heat;
  msg.robot_pos.position.x = robot_status.data.robot_pos_x;
  msg.robot_pos.position.y = robot_status.data.robot_pos_y;
  msg.robot_pos.orientation =
    tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_status.data.robot_pos_angle));
  //这个表示的是yaw角
  msg.armor_id = robot_status.data.armor_id;
  msg.hp_deduction_reason = robot_status.data.hp_deduction_reason;
  msg.projectile_allowance_17mm = robot_status.data.projectile_allowance_17mm;
  msg.remaining_gold_coin = robot_status.data.remaining_gold_coin;
  msg.powerful_offensive_posture=robot_status.data.powerful_offensive_posture;
  msg.powerful_defensive_posture=robot_status.data.powerful_defensive_posture;
  msg.powerful_movement_posture=robot_status.data.powerful_movement_posture;
  //ros2消息是被实时写入，在收到数据的线程中不断收到这些数据不断更新

  //if里面的msg.current_hp是相当于第一次将从下位机收取到的血量写进ros2消息中
  if (last_hp_ - msg.current_hp > 0) {
    msg.is_hp_deduced = true;
  }
  last_hp_ = robot_status.data.current_up;//last_hp_是第二次将从下位机收取到的血量数据存储进去

  robot_status_pub_->publish(msg);

  if (set_detector_color_) {
    uint8_t detect_color;
    if (getDetectColor(robot_status.data.robot_id, detect_color)) {
      if (!initial_set_param_ || detect_color != previous_receive_color_) {
        previous_receive_color_ = detect_color;
        setParam(rclcpp::Parameter("detect_color", detect_color));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
}

void StandardRobotPpRos2Node::publishJointState(ReceiveJointState & packet)
{
  last_gimbal_pitch_odom_joint_ = packet.data.pitch;
  last_gimbal_yaw_odom_joint_ = packet.data.yaw;
}

void StandardRobotPpRos2Node::publishBuff(ReceiveBuff & buff)
{
  pb_rm_interfaces::msg::Buff msg;
  msg.recovery_buff = buff.data.recovery_buff;
  msg.cooling_buff = buff.data.cooling_buff;
  msg.defence_buff = buff.data.defence_buff;
  msg.vulnerability_buff = buff.data.vulnerability_buff;
  msg.attack_buff = buff.data.attack_buff;
  msg.remaining_energy = buff.data.remaining_energy;
  buff_pub_->publish(msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData!");

  send_robot_cmd_data_.frame_header.sof = SOF_SEND;
  send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;
  send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 6;
  send_robot_cmd_data_.data.speed_vector.vx = 0;
  send_robot_cmd_data_.data.speed_vector.vy = 0;
  send_robot_cmd_data_.data.speed_vector.wz = 0;
  send_robot_cmd_data_.data.sentry_info.robot_posture = 3;
  send_robot_cmd_data_.data.sentry_info.switch_flag = 0;

  crc8::append_CRC8_check_sum(
    reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(HeaderFrame));

  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

   try {
      crc16::append_CRC16_check_sum(
        reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(SendRobotCmdData));
        std::vector<uint8_t> send_robot_cmd_data = toVector(send_robot_cmd_data_);
      serial_driver_->port()->send(send_robot_cmd_data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      is_usb_ok_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

// 👇 新增：姿态回调函数
void StandardRobotPpRos2Node::cmdPostureCallback(const pb_rm_interfaces::msg::SentryPosture::SharedPtr msg) {
    send_robot_cmd_data_.data.sentry_info.robot_posture = msg->current_posture;
}

//新增：哨兵旋转模式回调函数
void StandardRobotPpRos2Node::cmdSwitchFlagCallback(const pb_rm_interfaces::msg::SwitchFlag::SharedPtr msg){
  send_robot_cmd_data_.data.sentry_info.switch_flag = msg->switch_flag;
}

void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  send_robot_cmd_data_.data.speed_vector.vx = msg->linear.x;
  send_robot_cmd_data_.data.speed_vector.vy = msg->linear.y;
  send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z;
}//核心处理逻辑：对于订阅到的消息进行解码填入结构体send_robot_cmd_data_中


void StandardRobotPpRos2Node::setParam(const rclcpp::Parameter & param)
{
  if (!initial_set_param_) {
    auto node_graph = this->get_node_graph_interface();
    auto node_names = node_graph->get_node_names();
    std::vector<std::string> possible_detectors = {
      "armor_detector_openvino", "armor_detector_opencv"};

    for (const auto & name : possible_detectors) {
      for (const auto & node_name : node_names) {
        if (node_name.find(name) != std::string::npos) {
          detector_node_name_ = node_name;
          break;
        }
      }
      if (!detector_node_name_.empty()) {
        break;
      }
    }

    if (detector_node_name_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No detector node found!");
      return;
    }

    detector_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node_name_);
    if (!detector_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000, "Service not ready, skipping parameter set");
      return;
    }
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

bool StandardRobotPpRos2Node::getDetectColor(uint8_t robot_id, uint8_t & color)
{
  if (robot_id == 0 || (robot_id > 11 && robot_id < 101)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000, "Invalid robot ID: %d. Color not set.", robot_id);
    return false;
  }
  color = (robot_id >= 100) ? 0 : 1;
  return true;
}

bool StandardRobotPpRos2Node::callTriggerService(const std::string & service_name)
{
  auto client = this->create_client<std_srvs::srv::Trigger>(service_name);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto start_time = std::chrono::steady_clock::now();
  while (!client->wait_for_service(0.1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for the service: %s", service_name.c_str());
      return false;
    }
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (elapsed_time > std::chrono::seconds(5)) {
      RCLCPP_ERROR(
        get_logger(), "Service %s not available after 5 seconds, giving up.", service_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...", service_name.c_str());
  }

  auto result = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      get_logger(), "Service %s call succeeded: %s", service_name.c_str(),
      result.get()->success ? "true" : "false");
    return result.get()->success;
  }

  RCLCPP_ERROR(get_logger(), "Service %s call failed", service_name.c_str());
  return false;
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::StandardRobotPpRos2Node)
