# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the node"
    )
    declare_input_pose_is_imu = DeclareLaunchArgument(
        "input_pose_is_imu", default_value="true"
    )
    declare_lidar_translation = DeclareLaunchArgument(
        "lidar_in_imu_translation", default_value="[-0.011, -0.02329, 0.04412]"
    )
    declare_lidar_rotation = DeclareLaunchArgument(
        "lidar_in_imu_rotation",
        default_value="[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]",
    )

    start_loam_interface = Node(
        package="loam_interface",
        executable="loam_interface_node",
        name="loam_interface",
        namespace=namespace,
        remappings=remappings,
        output="screen",
        parameters=[
            {
                "state_estimation_topic": "aft_mapped_to_init",
                "registered_scan_topic": "cloud_registered",
                "odom_frame": "odom",
                "base_frame": "gimbal_yaw",
                "lidar_frame": "front_mid360",
                "input_pose_is_imu": ParameterValue(
                    LaunchConfiguration("input_pose_is_imu"), value_type=bool
                ),
                "lidar_in_imu_translation": LaunchConfiguration(
                    "lidar_in_imu_translation"
                ),
                "lidar_in_imu_rotation": LaunchConfiguration("lidar_in_imu_rotation"),
            }
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(declare_input_pose_is_imu)
    ld.add_action(declare_lidar_translation)
    ld.add_action(declare_lidar_rotation)
    ld.add_action(start_loam_interface)

    return ld
