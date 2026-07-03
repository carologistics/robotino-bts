cmake_minimum_required(VERSION 3.16)
project(robotino_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gigatino_msgs REQUIRED)
find_package(motor_move_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotino_vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(gigatino_msgs_typesupport gigatino_msgs::gigatino_msgs__rosidl_typesupport_cpp)

set(dependencies
  behaviortree_ros2
  btcpp_ros2_interfaces
  geometry_msgs
  gigatino_msgs
  motor_move_msgs
  nav2_msgs
  rclcpp
  robotino_vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set(action_node_list
  acquire_object_tracking
  calibrate_gripper
  motor_move
  move_gripper_to
  navigate_to
  set_gripper
)

foreach(action_node ${action_node_list})
  add_library(${action_node}_bt_node SHARED plugins/action/${action_node}.cpp)
  ament_target_dependencies(${action_node}_bt_node ${dependencies})
  if(TARGET ${gigatino_msgs_typesupport})
    target_link_libraries(${action_node}_bt_node ${gigatino_msgs_typesupport})
  endif()
  target_compile_definitions(${action_node}_bt_node PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs ${action_node}_bt_node)
endforeach()

set(service_node_list
  toggle_object_tracking
)

foreach(service_node ${service_node_list})
  add_library(${service_node}_bt_node SHARED plugins/service/${service_node}.cpp)
  ament_target_dependencies(${service_node}_bt_node ${dependencies})
  if(TARGET ${gigatino_msgs_typesupport})
    target_link_libraries(${service_node}_bt_node ${gigatino_msgs_typesupport})
  endif()
  target_compile_definitions(${service_node}_bt_node PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs ${service_node}_bt_node)
endforeach()

set(condition_node_list
  is_gripper_referenced
)

foreach(condition_node ${condition_node_list})
  add_library(${condition_node}_bt_node SHARED plugins/condition/${condition_node}.cpp)
  ament_target_dependencies(${condition_node}_bt_node ${dependencies})
  if(TARGET ${gigatino_msgs_typesupport})
    target_link_libraries(${condition_node}_bt_node ${gigatino_msgs_typesupport})
  endif()
  target_compile_definitions(${condition_node}_bt_node PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs ${condition_node}_bt_node)
endforeach()

add_executable(robotino_tree_server src/robotino_tree_server.cpp)
ament_target_dependencies(robotino_tree_server ${dependencies})

install(TARGETS
  robotino_tree_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(FILES robotino_behavior_tree.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
