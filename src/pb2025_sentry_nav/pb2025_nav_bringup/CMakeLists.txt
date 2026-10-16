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


install(DIRECTORY
  launch
  map
  pcd
  config
  behavior_trees
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()