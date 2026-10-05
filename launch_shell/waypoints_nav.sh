!/bin/bash
#export LD_PRELOAD="/lib/x86_64-linux-gnu/libpcl_*"
unset MVCAM_SDK_PATH

unset MVCAM_COMMON_RUNENV

unset MVCAM_GENICAM_CLPROTOCOL

unset ALLUSERSPROFILE

unset LD_LIBRARY_PATH

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH//:/opt\/MVS\/lib\/64:/}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH//:/opt\/MVS\/lib\/32:/}
export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libusb-1.0.so"
#colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# 启动 ROS 2
source /opt/ros/humble/setup.bash
source /home/vgd/code/ros_ws/install/setup.sh

ros2 launch waypoint_editor waypoint_editor.launch.py
