cmake_minimum_required(VERSION 3.5)
project(pb2025_robot_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(body_static_tf_broadcaster src/body_static_tf_broadcaster.cpp)
target_compile_features(body_static_tf_broadcaster PRIVATE cxx_std_14)
target_compile_options(body_static_tf_broadcaster PRIVATE -Wall -Wextra -Werror)
ament_target_dependencies(body_static_tf_broadcaster rclcpp geometry_msgs tf2 tf2_ros)
install(TARGETS body_static_tf_broadcaster DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    params
    resource
    rviz

    DESTINATION share/${PROJECT_NAME}/
)

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

ament_package()
