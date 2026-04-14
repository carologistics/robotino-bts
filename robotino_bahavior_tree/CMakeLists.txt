cmake_minimum_required(VERSION 3.8)
project(robotino_bahavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gigatino_msgs REQUIRED)
find_package(motor_move_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  behaviortree_cpp_v3
  geometry_msgs
  gigatino_msgs
  motor_move_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
)

include_directories(
  include
)

set(action_node_list
  motor_move
  move_gripper_to
)

foreach(action_node ${action_node_list})
  add_library(${action_node}_bt_node SHARED plugins/action/${action_node}.cpp)
  ament_target_dependencies(${action_node}_bt_node ${dependencies})
  target_compile_definitions(${action_node}_bt_node PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs ${action_node}_bt_node)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES robotino_bahavior_tree.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})

ament_package()
