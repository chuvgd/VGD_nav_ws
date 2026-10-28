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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{
namespace
{
// 检验给定的Position是否是有限值
template <typename Vector>
bool finitePosition(const Vector & position)
{
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
}

// 四元数归一化
bool normalizeQuaternion(geometry_msgs::msg::Quaternion & q)
{
  // 先检验是否为有限值
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
    return false;
  }
  // 归一化具体逻辑
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || norm < 1e-12) {
    return false;
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return true;
}

// 检验给定的坐标变换是否为有限值
bool finiteTransform(const tf2::Transform & transform)
{
  return finitePosition(tf2::toMsg(transform).translation) &&
         std::isfinite(transform.getRotation().length2());
}

bool validStamp(const builtin_interfaces::msg::Time & stamp)
{
  // Reject malformed stamps before constructing rclcpp::Time (negative seconds throw).
  return stamp.sec >= 0 && stamp.nanosec < 1000000000u && (stamp.sec != 0 || stamp.nanosec != 0u);
}

}  // namespace

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  min_dt_ = this->declare_parameter<double>("min_dt", 1e-4);
  max_dt_ = this->declare_parameter<double>("max_dt", 0.5);
  max_sync_skew_ = this->declare_parameter<double>("max_sync_skew", 0.05);
  transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.05);
  if (lidar_frame_.empty() || base_frame_.empty() || robot_base_frame_.empty()) {
    throw std::invalid_argument("lidar_frame, base_frame and robot_base_frame must be nonempty");
  }
  if (
    !std::isfinite(min_dt_) || !std::isfinite(max_dt_) || min_dt_ <= 0.0 || max_dt_ <= min_dt_ ||
    !std::isfinite(max_sync_skew_) || max_sync_skew_ < 0.0 || !std::isfinite(transform_timeout_) ||
    transform_timeout_ < 0.0) {
    throw std::invalid_argument("Invalid time limits for sensor_scan_generation");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  if (!validStamp(odometry_msg->header.stamp) || !validStamp(pcd_msg->header.stamp)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Dropping invalid/zero sensor stamp");
    has_previous_pose_ = false;
    return;
  }
  // The pose is measured at the odometry stamp, regardless of callback scheduling or /clock rate.
  const rclcpp::Time stamp(odometry_msg->header.stamp);
  const rclcpp::Time cloud_stamp(pcd_msg->header.stamp);
  if (
    std::abs((stamp - cloud_stamp).seconds()) > max_sync_skew_ ||
    odometry_msg->header.frame_id.empty() || odometry_msg->child_frame_id != lidar_frame_ ||
    pcd_msg->header.frame_id != odometry_msg->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropping scan/odometry pair with invalid stamps, excessive skew or inconsistent frames");
    has_previous_pose_ = false;
    return;
  }

  if (has_previous_pose_ && previous_parent_frame_ == odometry_msg->header.frame_id) {
    const double dt = (stamp - previous_stamp_).seconds();
    if (dt >= 0.0 && dt < min_dt_) {
      // Duplicate/too-close samples must not replace the last useful difference baseline.
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping duplicate/too-close odometry stamp");
      return;
    }
    if (dt < 0.0 || dt > max_dt_) {
      // A backwards stamp starts a new epoch (e.g. bag seek or simulation reset).
      has_previous_pose_ = false;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Resetting velocity history after time jump/gap");
    }
  } else {
    has_previous_pose_ = false;
  }

  auto pose = odometry_msg->pose.pose;
  if (!finitePosition(pose.position) || !normalizeQuaternion(pose.orientation)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Dropping invalid odometry pose/quaternion");
    has_previous_pose_ = false;
    return;
  }

  tf2::Transform tf_lidar_to_robot_base;
  tf2::Transform tf_lidar_to_chassis;
  tf2::Transform tf_odom_to_chassis;
  tf2::Transform tf_odom_to_robot_base;
  tf2::Transform tf_odom_to_lidar;

  tf2::fromMsg(pose, tf_odom_to_lidar);
  if (
    !getTransform(lidar_frame_, robot_base_frame_, stamp, tf_lidar_to_robot_base) ||
    !getTransform(lidar_frame_, base_frame_, stamp, tf_lidar_to_chassis)) {
    has_previous_pose_ = false;
    return;
  }

  tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base;
  if (!finiteTransform(tf_odom_to_chassis) || !finiteTransform(tf_odom_to_robot_base)) {
    has_previous_pose_ = false;
    return;
  }

  publishTransform(tf_odom_to_chassis, odometry_msg->header.frame_id, base_frame_, stamp);
  publishOdometry(tf_odom_to_robot_base, odometry_msg->header.frame_id, robot_base_frame_, stamp);

  sensor_msgs::msg::PointCloud2 out;
  pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out);
  pub_laser_cloud_->publish(out);
}

bool SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  tf2::Transform & transform)
{
  try {
    // Bound TF waiting with wall time: a paused simulation clock must not stall this callback.
    // The TransformListener services its own thread while we wait.
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(transform_timeout_);
    // 不能转换就等待
    while (!tf_buffer_->canTransform(target_frame, source_frame, time) &&
           std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // 直到能进行转换后进行坐标转换
    auto transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, time);
    if (
      !finitePosition(transform_stamped.transform.translation) ||
      !normalizeQuaternion(transform_stamped.transform.rotation)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Dropping invalid TF %s <- %s",
        target_frame.c_str(), source_frame.c_str());
      return false;
    }
    tf2::fromMsg(transform_stamped.transform, transform);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "TF lookup failed; dropping pair: %s",
      ex.what());
    return false;
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  auto rotation = transform.getRotation();
  rotation.normalize();
  if (has_previous_pose_ && rotation.dot(previous_chassis_orientation_) < 0.0) {
    rotation *= -1.0;
  }
  previous_chassis_orientation_ = rotation;
  transform_msg.transform.rotation = tf2::toMsg(rotation);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  auto orientation = transform.getRotation();
  orientation.normalize();
  // 双倍覆盖，如果路径大就进行取反
  if (has_previous_pose_ && orientation.dot(previous_orientation_) < 0.0) {
    orientation *= -1.0;
  }
  out.pose.pose.orientation = tf2::toMsg(orientation);

  if (has_previous_pose_) {
    const double dt = (stamp - previous_stamp_).seconds();
    // Preserve this repository's odom-frame feedback convention for the fake-yaw Nav2 setup.
    // child_frame_id still identifies the pose frame used by fake_vel_transform to read yaw.
    // This intentionally differs from the standard Odometry child-frame twist convention.
    const auto linear_velocity = (origin - previous_transform_.getOrigin()) / dt;

    auto q_diff = orientation * previous_orientation_.inverse();
    q_diff.normalize();
    // q and -q describe the same rotation. Use the shortest arc, including across +/-pi.
    if (q_diff.w() < 0.0) {
      q_diff *= -1.0;
    }
    const tf2::Vector3 rotation_vector(q_diff.x(), q_diff.y(), q_diff.z());
    const double sin_half_angle = rotation_vector.length();
    const double scale = sin_half_angle < 1e-12
                           ? 2.0 / dt
                           : 2.0 * std::atan2(sin_half_angle, q_diff.w()) / (sin_half_angle * dt);
    const auto angular_velocity = rotation_vector * scale;

    out.twist.twist.linear.x = linear_velocity.x();
    out.twist.twist.linear.y = linear_velocity.y();
    out.twist.twist.linear.z = linear_velocity.z();
    out.twist.twist.angular.x = angular_velocity.x();
    out.twist.twist.angular.y = angular_velocity.y();
    out.twist.twist.angular.z = angular_velocity.z();
  } else {
    // A single pose cannot measure velocity. Publish zero with explicitly unknown uncertainty.
    for (size_t i = 0; i < 6; ++i) {
      out.twist.covariance[i * 6 + i] = 1e6;
    }
  }

  previous_transform_ = transform;
  previous_orientation_ = orientation;
  previous_stamp_ = stamp;
  previous_parent_frame_ = parent_frame;
  has_previous_pose_ = true;

  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
