cmake_minimum_required(VERSION 3.16)
project(pb2025_sentry_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_executable(pb2025_sentry_behavior_server src/pb2025_sentry_behavior_server.cpp)

ament_auto_add_library(action_client SHARED
  src/pb2025_sentry_behavior_client.cpp
)

rclcpp_components_register_node(action_client
  PLUGIN pb2025_sentry_behavior::SentryBehaviorClient
  EXECUTABLE pb2025_sentry_behavior_client
)

####### Conditions #######
ament_auto_add_library(is_game_status SHARED plugins/condition/is_game_status.cpp)
list(APPEND plugin_libs is_game_status)

ament_auto_add_library(is_flag_set SHARED plugins/condition/is_flag_set.cpp)
list(APPEND plugin_libs is_flag_set)

ament_auto_add_library(is_last_waypoint SHARED plugins/condition/is_last_waypoint.cpp)
list(APPEND plugin_libs is_last_waypoint)

ament_auto_add_library(is_enemy_in_range SHARED plugins/condition/is_enemy_in_range.cpp)
list(APPEND plugin_libs is_enemy_in_range)

ament_auto_add_library(is_power_posture_available SHARED plugins/condition/is_power_posture_available.cpp)
list(APPEND plugin_libs is_power_posture_available)

ament_auto_add_library(is_rfid_detected SHARED plugins/condition/is_rfid_detected.cpp)
list(APPEND plugin_libs is_rfid_detected)

ament_auto_add_library(is_status_ok SHARED plugins/condition/is_status_ok.cpp)
list(APPEND plugin_libs is_status_ok)

ament_auto_add_library(is_attacked SHARED plugins/condition/is_attacked.cpp)
list(APPEND plugin_libs is_attacked)

ament_auto_add_library(is_detect_enemy SHARED plugins/condition/is_detect_enemy.cpp)
list(APPEND plugin_libs is_detect_enemy)

ament_auto_add_library(is_cmd_vel_zero SHARED plugins/condition/is_cmd_vel_zero.cpp)
list(APPEND plugin_libs is_cmd_vel_zero)

ament_auto_add_library(is_goal_reached SHARED plugins/condition/is_goal_reached.cpp)
list(APPEND plugin_libs is_goal_reached)

ament_auto_add_library(is_enemy_outpost_dead SHARED plugins/condition/is_enemy_outpost_dead.cpp)
list(APPEND plugin_libs is_enemy_outpost_dead)

####### Actions #######
ament_auto_add_library(send_nav2_goal SHARED plugins/action/send_nav2_goal.cpp)
list(APPEND plugin_libs send_nav2_goal)

ament_auto_add_library(pub_cmdvel SHARED plugins/action/pub_cmdvel.cpp)
list(APPEND plugin_libs pub_cmdvel)

ament_auto_add_library(switch_sentry_posture SHARED plugins/action/switch_sentry_posture.cpp)
list(APPEND plugin_libs switch_sentry_posture)

ament_auto_add_library(pub_nav2_goal SHARED plugins/action/pub_nav2_goal.cpp)
list(APPEND plugin_libs pub_nav2_goal)

ament_auto_add_library(pub_nav2_goal_bb SHARED plugins/action/pub_nav2_goal_bb.cpp)
list(APPEND plugin_libs pub_nav2_goal_bb)

ament_auto_add_library(pub_spin_speed SHARED plugins/action/pub_spin_speed.cpp)
list(APPEND plugin_libs pub_spin_speed)

ament_auto_add_library(pub_twist SHARED plugins/action/pub_twist.cpp)
list(APPEND plugin_libs pub_twist)

ament_auto_add_library(pub_gimbal_absolute SHARED plugins/action/pub_gimbal_absolute.cpp)
list(APPEND plugin_libs pub_gimbal_absolute)

ament_auto_add_library(pub_gimbal_velocity SHARED plugins/action/pub_gimbal_velocity.cpp)
list(APPEND plugin_libs pub_gimbal_velocity)

ament_auto_add_library(calculate_attack_pose SHARED plugins/action/calculate_attack_pose.cpp)
list(APPEND plugin_libs calculate_attack_pose)

ament_auto_add_library(load_waypoints SHARED plugins/action/load_waypoints.cpp)
list(APPEND plugin_libs load_waypoints)

ament_auto_add_library(get_current_waypoint SHARED plugins/action/get_current_waypoint.cpp)
list(APPEND plugin_libs get_current_waypoint)

ament_auto_add_library(wait_until_reached SHARED plugins/action/wait_until_reached.cpp)
list(APPEND plugin_libs wait_until_reached)

ament_auto_add_library(wait_duration SHARED plugins/action/wait_duration.cpp)
list(APPEND plugin_libs wait_duration)

ament_auto_add_library(next_waypoint SHARED plugins/action/next_waypoint.cpp)
list(APPEND plugin_libs next_waypoint)

ament_auto_add_library(switch_flag SHARED plugins/action/switch_flag.cpp)
list(APPEND plugin_libs switch_flag)
####### Controls #######
ament_auto_add_library(recovery_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs recovery_node)

####### Decorators #######
ament_auto_add_library(rate_controller SHARED plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs rate_controller)

ament_auto_add_library(tick_after_timeout_node SHARED plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_node)

# ament_auto_add_library(delay_after_success SHARED plugins/decorator/delay_after_success.cpp)
# list(APPEND plugin_libs delay_after_success)

ament_auto_add_library(delay_after_success SHARED plugins/decorator/delay_after_success.cpp)
list(APPEND plugin_libs delay_after_success)

ament_auto_add_library(retry_until_success SHARED plugins/decorator/retry_until_success.cpp)
list(APPEND plugin_libs retry_until_success)


foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(
  TARGETS ${plugin_libs}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

ament_auto_package(
  INSTALL_TO_SHARE
  behavior_trees
  params
  launch
)
