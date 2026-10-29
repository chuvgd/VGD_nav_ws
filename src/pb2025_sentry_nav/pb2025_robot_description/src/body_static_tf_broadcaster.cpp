#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

// Only robot-internal installation transforms. Navigation owns map/odom and fake-yaw TF.
class BodyStaticTfBroadcaster : public rclcpp::Node
{
public:
  BodyStaticTfBroadcaster() : Node("body_static_tf_broadcaster")
  {
    const double chassis_z = declare_parameter<double>("chassis_height", 0.077);
    const double gimbal_z = declare_parameter<double>("gimbal_height", 0.026);
    const auto lidar_xyz = declare_parameter<std::vector<double>>(
      "lidar_xyz", std::vector<double>{-0.056, -0.213, 0.240});
    const auto lidar_rpy = declare_parameter<std::vector<double>>(
      "lidar_rpy", std::vector<double>{0.7853981633974483, 0.0, 0.0});
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(makeTransform("base_footprint", "chassis", {0.0, 0.0, chassis_z}));
    transforms.push_back(makeTransform("chassis", "gimbal_yaw", {0.0, 0.0, gimbal_z}));
    transforms.push_back(makeTransform("gimbal_yaw", "front_mid360", lidar_xyz, lidar_rpy));
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(get_logger(), "Published 3 body/LiDAR static transforms");
  }

private:
  geometry_msgs::msg::TransformStamped makeTransform(
    const std::string & parent, const std::string & child, const std::vector<double> & xyz,
    const std::vector<double> & rpy = {0.0, 0.0, 0.0})
  {
    if (xyz.size() != 3 || rpy.size() != 3) {
      throw std::invalid_argument("Installation xyz/rpy must contain exactly three doubles");
    }
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(xyz[i]) || !std::isfinite(rpy[i])) {
        throw std::invalid_argument("Installation xyz/rpy must be finite");
      }
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = xyz[0];
    transform.transform.translation.y = xyz[1];
    transform.transform.translation.z = xyz[2];
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    q.normalize();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    return transform;
  }

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BodyStaticTfBroadcaster>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("body_static_tf_broadcaster"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
