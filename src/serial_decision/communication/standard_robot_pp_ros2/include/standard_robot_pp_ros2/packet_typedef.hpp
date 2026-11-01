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

#ifndef STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
#define STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace standard_robot_pp_ros2
{
const uint8_t SOF_RECEIVE = 0x5A;
const uint8_t SOF_SEND = 0x5A;

// Receive
const uint8_t ID_DEBUG = 0x01;
const uint8_t ID_IMU = 0x02;
const uint8_t ID_ROBOT_STATE_INFO = 0x03;
const uint8_t ID_EVENT_DATA = 0x04;
const uint8_t ID_PID_DEBUG = 0x05;
const uint8_t ID_ALL_ROBOT_HP = 0x06;
const uint8_t ID_GAME_STATUS = 0x07;
const uint8_t ID_ROBOT_MOTION = 0x08;
const uint8_t ID_GROUND_ROBOT_POSITION = 0x09;
const uint8_t ID_RFID_STATUS = 0x0A;
const uint8_t ID_ROBOT_STATUS = 0x0B;
const uint8_t ID_JOINT_STATE = 0x0C;
const uint8_t ID_BUFF = 0x0D;
// Send
// 在 packet_typedef.hpp 的 "Send" 注释区域添加
const uint8_t ID_ROBOT_CMD = 0x01;


const uint8_t DEBUG_PACKAGE_NUM = 10;
const uint8_t DEBUG_PACKAGE_NAME_LEN = 10;

struct HeaderFrame
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/
// 事件数据包
struct ReceiveEventData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t supply_zone : 1;

    uint8_t small_energy : 1;
    uint8_t big_energy : 1;

    uint8_t R_outpost : 3;
    uint8_t B_outpost : 3;

    uint8_t R_outpost_control : 2;
    uint8_t B_outpost_control : 2;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));


// 全场机器人hp信息数据包
struct ReceiveAllRobotHpData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    // uint16_t red_1_robot_hp;
    // uint16_t red_2_robot_hp;
    // uint16_t red_3_robot_hp;
    // uint16_t red_4_robot_hp;
    // uint16_t red_7_robot_hp;
    // uint16_t red_outpost_hp;
    // uint16_t red_base_hp;

    // uint16_t blue_1_robot_hp;
    // uint16_t blue_2_robot_hp;
    // uint16_t blue_3_robot_hp;
    // uint16_t blue_4_robot_hp;
    // uint16_t blue_7_robot_hp;
    // uint16_t blue_outpost_hp;
    // uint16_t blue_base_hp;
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    int16_t damage_difference;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 比赛信息数据包
struct ReceiveGameStatusData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t game_progress;
    uint16_t stage_remain_time;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// RFID 状态数据包
struct ReceiveRfidStatus
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t ally_supply_point : 1;
    uint8_t R_outpost_point : 1;
    uint8_t B_outpost_point : 1;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// 机器人状态数据包
struct ReceiveRobotStatus
{
  HeaderFrame frame_header;

  uint32_t time_stamp;

  struct
  {
    uint16_t current_up;
    uint16_t maximum_hp;
    uint16_t projectile_allowance_17mm;
    uint16_t remaining_gold_coin;

    //能否进入强化姿态
    uint8_t powerful_offensive_posture;
    uint8_t powerful_defensive_posture;
    uint8_t powerful_movement_posture;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// 建筑物血量数据包
struct ReceiveBuildingStatus
{
  HeaderFrame frame_header;

  uint32_t time_stamp;

  struct{
    uint16_t outpost_R_HP;
    uint16_t ally_base_HP;
    uint16_t outpost_B_HP;
    uint16_t enemy_base_HP;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

/********************************************************/
/* Send data                                            */
/********************************************************/

struct SendRobotCmdData
{
  HeaderFrame frame_header;

  uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) speed_vector;//下位机收到速度指令重要结构体，发nav2控制器输出的速度给下位机

    struct
    {
      uint8_t robot_posture;
    }__attribute__((packed)) sentry_info;
  } __attribute__((packed)) data;

  uint16_t checksum;
} __attribute__((packed));

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
