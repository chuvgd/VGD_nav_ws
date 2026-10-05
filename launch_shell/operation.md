# 给脚本执行权限
chmod +x /home/vgd/code/ros_ws/sentry_launch.sh

# 重新加载systemd配置
sudo systemctl daemon-reload

# 启用开机自启动
sudo systemctl enable sentry-launch.service

# 立即启动服务（可选）
sudo systemctl start sentry-launch.service

# 查看服务状态
sudo systemctl status sentry-launch.service

# 查看日志
sudo journalctl -u sentry-launch.service -f

# 停止正在运行的服务
sudo systemctl stop sentry-launch.service

# 禁用开机自启动
sudo systemctl disable sentry-launch.service

# 确认服务状态（应该是 disabled）
sudo systemctl status sentry-launch.service

# 检查是否已禁用
sudo systemctl is-enabled sentry-launch.service

保存栅格地图：ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1

# 导航模式
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False \
use_robot_state_pub:=True

ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \ use_rviz:=True

ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py

# 停止服务（进程会被终止）
sudo systemctl stop robomaster.service

# 查看状态确认已停止
sudo systemctl status robomaster.service

# 保存地图
ros2 run nav2_map_server map_saver_cli -f VGD

#打印速度日志
ros2 topic echo /cmd_vel > test2.txt
