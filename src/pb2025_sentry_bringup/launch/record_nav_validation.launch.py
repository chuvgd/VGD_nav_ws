"""Record only raw MID360 LiDAR/IMU inputs for offline comparison."""
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def setup_recording(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    prefix = "/" + namespace if namespace else ""
    topics = ["livox/lidar", "livox/imu"]
    topics = [prefix + "/" + topic for topic in topics]
    return [
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-o",
                LaunchConfiguration("output"),
                *topics,
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "output",
                default_value="rosbags/nav_validation_"
                + datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            ),
            OpaqueFunction(function=setup_recording),
        ]
    )
