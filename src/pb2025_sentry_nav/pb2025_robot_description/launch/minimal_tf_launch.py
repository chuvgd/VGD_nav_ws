from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("pb2025_robot_description")
    profile = LaunchConfiguration("robot_name")
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("robot_name", default_value="VGD2027_sentry_robot"),
            DeclareLaunchArgument(
                "tf_params_file",
                default_value=PathJoinSubstitution(
                    [share, "params", [profile, "_body_tf.yaml"]]
                ),
            ),
            Node(
                package="pb2025_robot_description",
                executable="body_static_tf_broadcaster",
                name="body_static_tf_broadcaster",
                # Absolute namespace also works inside the existing serial/navigation namespace groups.
                namespace=PathJoinSubstitution(["/", LaunchConfiguration("namespace")]),
                parameters=[
                    LaunchConfiguration("tf_params_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    },
                ],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                output="screen",
            ),
        ]
    )
