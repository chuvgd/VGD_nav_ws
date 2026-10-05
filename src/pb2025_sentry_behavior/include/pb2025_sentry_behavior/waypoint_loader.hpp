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

#ifndef PB2025_SENTRY_BEHAVIOR__WAYPOINT_LOADER_HPP_
#define PB2025_SENTRY_BEHAVIOR__WAYPOINT_LOADER_HPP_

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief 从 waypoint_editor 生成的 CSV 文件中加载航点列表
 *
 * CSV 格式: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,wait_sec
 * wait_sec 列可选，缺省为 0（不等待）
 *
 * @param filename CSV 文件的绝对路径
 * @param waypoints 输出：航点列表
 * @param frame_id 坐标系 ID，默认 "map"
 * @param wait_times 输出：每个航点的等待时间（秒），可为 nullptr 表示不需要
 * @return true 加载成功，false 加载失败
 */
inline bool loadWaypointsFromCSV(
  const std::string & filename,
  std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  const std::string & frame_id = "map",
  std::vector<double> * wait_times = nullptr)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  // 跳过 CSV 表头
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) continue;

    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    // 至少需要 8 列: id, pose_x, pose_y, pose_z, rot_x, rot_y, rot_z, rot_w
    if (tokens.size() < 8) {
      continue;
    }

    try {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp = rclcpp::Time(0);

      pose.pose.position.x = std::stod(tokens[1]);
      pose.pose.position.y = std::stod(tokens[2]);
      pose.pose.position.z = std::stod(tokens[3]);

      pose.pose.orientation.x = std::stod(tokens[4]);
      pose.pose.orientation.y = std::stod(tokens[5]);
      pose.pose.orientation.z = std::stod(tokens[6]);
      pose.pose.orientation.w = std::stod(tokens[7]);

      waypoints.push_back(pose);

      // 第 10 列 (index 9) 为 wait_sec，可选
      if (wait_times) {
        double wt = 0.0;
        if (tokens.size() > 9 && !tokens[9].empty()) {
          wt = std::stod(tokens[9]);
        }
        wait_times->push_back(wt);
      }
    } catch (const std::exception &) {
      continue;
    }
  }

  file.close();
  return !waypoints.empty();
}

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__WAYPOINT_LOADER_HPP_
