cmake_minimum_required(VERSION 3.8)
project(pb2025_nav_bringup)

# 默认使用C++17标准（ROS2 Humble 推荐）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找ament_cmake（必须在所有ROS包之前）
find_package(ament_cmake REQUIRED)

# 查找其他必要的包
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 可选：添加测试（如果BUILD_TESTING被启用）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

# 创建可执行文件
add_executable(auto_high_ground_navigator 
  src/auto_high_ground_navigator.cpp
)

# 方法1：使用 ament_target_dependencies（推荐）
ament_target_dependencies(auto_high_ground_navigator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_msgs
  std_srvs
)

# 方法2：或者使用传统变量链接方式（如果方法1不行）
# target_link_libraries(auto_high_ground_navigator
#   ${rclcpp_LIBRARIES}
#   ${rclcpp_action_LIBRARIES}
#   ${geometry_msgs_LIBRARIES}
#   ${nav2_msgs_LIBRARIES}
#   ${std_msgs_LIBRARIES}
#   ${std_srvs_LIBRARIES}
# )

# 包含头文件目录
target_include_directories(auto_high_ground_navigator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS auto_high_ground_navigator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件和其他资源（保持原有）
install(DIRECTORY
  launch
  map
  pcd
  config
  behavior_trees
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖（使用ament_export_targets更安全）
#ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 或者使用传统的export方式
#ament_export_dependencies(rclcpp)
#ament_export_dependencies(rclcpp_action)
#ament_export_dependencies(geometry_msgs)
#ament_export_dependencies(nav2_msgs)
#ament_export_dependencies(std_msgs)
#ament_export_dependencies(std_srvs)

# 自动生成环境钩子和安装目标
ament_package()