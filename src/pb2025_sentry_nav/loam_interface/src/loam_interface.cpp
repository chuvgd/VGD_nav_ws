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

#include "loam_interface/loam_interface.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace loam_interface
{

LoamInterfaceNode::LoamInterfaceNode(const rclcpp::NodeOptions & options)
: Node("loam_interface", options)
{
  this->declare_parameter<std::string>("state_estimation_topic", "");
  this->declare_parameter<std::string>("registered_scan_topic", "");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("lidar_frame", "");

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);

  input_pose_is_imu_ = this->declare_parameter<bool>("input_pose_is_imu", false);
  const auto translation = this->declare_parameter<std::vector<double>>(
    "lidar_in_imu_translation", std::vector<double>{0.0, 0.0, 0.0});
  const auto rotation = this->declare_parameter<std::vector<double>>(
    "lidar_in_imu_rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  if (translation.size() != 3 || rotation.size() != 9) {
    throw std::invalid_argument("LiDAR-in-IMU translation/rotation must have 3/9 elements");
  }
  for (const double value : translation) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("LiDAR-in-IMU translation must be finite");
    }
  }
  for (const double value : rotation) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("LiDAR-in-IMU rotation must be finite");
    }
  }
  const tf2::Matrix3x3 basis(
    rotation[0], rotation[1], rotation[2], rotation[3], rotation[4], rotation[5], rotation[6],
    rotation[7], rotation[8]);
  const auto orthogonality = basis.transpose() * basis;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (std::abs(orthogonality[row][col] - (row == col ? 1.0 : 0.0)) > 1e-6) {
        throw std::invalid_argument("LiDAR-in-IMU rotation must be orthonormal");
      }
    }
  }
  if (std::abs(basis.determinant() - 1.0) > 1e-6) {
    throw std::invalid_argument("LiDAR-in-IMU rotation must have determinant +1");
  }
  tf_imu_to_lidar_.setBasis(basis);
  tf_imu_to_lidar_.setOrigin(tf2::Vector3(translation[0], translation[1], translation[2]));

  base_frame_to_lidar_initialized_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, 5,
    std::bind(&LoamInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_estimation_topic_, 5,
    std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));
}

void LoamInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // NOTE: Input point cloud message is based on the `lidar_odom`
  // Here we transform it to the REAL `odom` frame
  if (!base_frame_to_lidar_initialized_) {
    return;
  }
  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_lidar_odom_, *msg, *out);
  pcd_pub_->publish(*out);
}

void LoamInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // NOTE: Input odometry message is based on the `lidar_odom`
  // Here we transform it to the `odom` frame
  if (!base_frame_to_lidar_initialized_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);
      tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar;
      if (input_pose_is_imu_) {
        // A = C * inverse(E). Use the same world alignment for pose and registered points.
        // Keep the existing initial-world convention: no first-pose/gravity normalization here.
        tf_odom_to_lidar_odom_ *= tf_imu_to_lidar_.inverse();
      }
      base_frame_to_lidar_initialized_ = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      return;
    }
  }

  // Transform the odometry_msg (based lidar_odom) to the odom frame
  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar);
  tf2::Transform tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;
  if (input_pose_is_imu_) {
    // The LIO state is T_WI; right-multiplying E makes the output a LiDAR pose T_OL.
    // Combined with A above: T_OL = C * inverse(E) * T_WI * E.
    tf_odom_to_lidar *= tf_imu_to_lidar_;
  }

  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = tf_odom_to_lidar.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());

  odom_pub_->publish(out);
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
