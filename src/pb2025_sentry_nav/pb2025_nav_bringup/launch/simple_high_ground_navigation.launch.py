#!/usr/bin/env python3
"""
简化版高地导航启动文件
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_pkg = FindPackageShare('pb2025_nav_bringup')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([bringup_pkg, 'launch', 'navigation_with_high_ground.launch.py'])
            ]),
            launch_arguments={
                'slam': 'false',  # 使用定位模式
                'map': PathJoinSubstitution([bringup_pkg, 'maps', 'map.yaml']),
                'enable_high_ground_nav': 'true',
                'use_sim_time': 'false',
            }.items()
        )
    ])