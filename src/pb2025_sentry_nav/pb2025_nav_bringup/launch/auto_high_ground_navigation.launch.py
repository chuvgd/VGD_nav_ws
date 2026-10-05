#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('pb2025_nav_bringup')
    
    # 定义启动参数
    declare_enabled = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='是否启用自动高地导航'
    )
    
    declare_high_ground_x = DeclareLaunchArgument(
        'high_ground_x',
        default_value='10.0',
        description='高地X坐标'
    )
    
    declare_high_ground_y = DeclareLaunchArgument(
        'high_ground_y',
        default_value='8.0',
        description='高地Y坐标'
    )
    
    declare_high_ground_z = DeclareLaunchArgument(
        'high_ground_z',
        default_value='0.0',
        description='高地Z坐标'
    )
    
    declare_high_ground_orientation_w = DeclareLaunchArgument(
        'high_ground_orientation_w',
        default_value='1.0',
        description='高地朝向W分量'
    )
    
    declare_delay_before_start = DeclareLaunchArgument(
        'delay_before_start',
        default_value='1.0',
        description='启动后延迟时间（秒）'
    )
    
    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='是否自动开始导航'
    )
    
    # 关键参数：动作服务器名称（根据您的系统修改）
    declare_action_server_name = DeclareLaunchArgument(
        'action_server_name',
        default_value='/red_standard_robot1/navigate_to_pose',
        description='导航动作服务器名称'
    )
    
    declare_server_timeout = DeclareLaunchArgument(
        'server_timeout',
        default_value='15.0',
        description='服务器连接超时时间（秒）'
    )
    
    declare_max_retry_attempts = DeclareLaunchArgument(
        'max_retry_attempts',
        default_value='5',
        description='最大重试次数'
    )
    
    # 自动高地导航节点
    auto_high_ground_node = Node(
        package='pb2025_nav_bringup',
        executable='auto_high_ground_navigator',
        name='auto_high_ground_navigator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'enabled': LaunchConfiguration('enabled'),
            'high_ground_x': LaunchConfiguration('high_ground_x'),
            'high_ground_y': LaunchConfiguration('high_ground_y'),
            'high_ground_z': LaunchConfiguration('high_ground_z'),
            'high_ground_orientation_w': LaunchConfiguration('high_ground_orientation_w'),
            'delay_before_start': LaunchConfiguration('delay_before_start'),
            'auto_start': LaunchConfiguration('auto_start'),
            'action_server_name': LaunchConfiguration('action_server_name'),
            'server_timeout': LaunchConfiguration('server_timeout'),
            'max_retry_attempts': LaunchConfiguration('max_retry_attempts'),
        }]
    )
    
    return LaunchDescription([
        declare_enabled,
        declare_high_ground_x,
        declare_high_ground_y,
        declare_high_ground_z,
        declare_high_ground_orientation_w,
        declare_delay_before_start,
        declare_auto_start,
        declare_action_server_name,
        declare_server_timeout,
        declare_max_retry_attempts,
        auto_high_ground_node,
    ])