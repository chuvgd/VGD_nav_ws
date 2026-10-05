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
#colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10

# 加载 ROS 2 环境
source ~/code/ros_ws/install/setup.bash

# 启动导航all
ros2 launch pb2025_sentry_bringup bringup.launch.py \
world:=VGD \
use_rviz:=True \
use_robot_state_pub:=True \
#use_composition:=False
