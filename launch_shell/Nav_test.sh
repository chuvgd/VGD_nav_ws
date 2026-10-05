#!/bin/bash
#export LD_PRELOAD="/lib/x86_64-linux-gnu/libpcl_*"
unset MVCAM_SDK_PATH
unset MVCAM_COMMON_RUNENV
unset MVCAM_GENICAM_CLPROTOCOL
unset ALLUSERSPROFILE
unset LD_LIBRARY_PATH

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH//:/opt\/MVS\/lib\/64:/}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH//:/opt\/MVS\/lib\/32:/}
export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libusb-1.0.so"

# 编译（若无需每次编译可注释掉下一行）
#colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载 ROS 2 环境
source /opt/ros/humble/setup.bash
source /home/vgd/code/ros_ws/install/setup.sh



# 1. 启动导航系统（后台运行）
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    world:=VGD \
    slam:=False \
    use_robot_state_pub:=True &
NAV_PID=$!
echo "导航系统启动中，PID: $NAV_PID"

# 等待导航系统基本就绪
echo "等待导航系统启动..."
sleep 8

# 2. 启动标准机器人控制（后台运行）
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py &
CONTROL_PID=$!
echo "机器人控制启动中，PID: $CONTROL_PID"

# 等待控制启动
echo "等待机器人控制启动..."
sleep 5
