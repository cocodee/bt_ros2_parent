cmake_minimum_required(VERSION 3.16)
project(bt_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# ----------------------------------------

# Assuming your code is being built as a library
add_library(behavior_tree_node_lib
  src/behavior_tree_node.cpp
)
# Include directories
include_directories(include)

# Custom Action Node: PrintCommand
add_library(print_command_action
  src/print_command_action.cpp
)
target_include_directories(print_command_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(print_command_action
  behaviortree_cpp_v3
  rclcpp
)


target_include_directories(behavior_tree_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(behavior_tree_node_lib
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  my_robot_interfaces
  nav2_behavior_tree
  controller_manager_msgs
)
target_link_libraries(behavior_tree_node_lib
  print_command_action
)

# Main Executable
add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor rclcpp)
target_link_libraries(bt_executor behavior_tree_node_lib)

# Install rules
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY bt_trees
  DESTINATION share/${PROJECT_NAME}/
)
install(
  TARGETS
  print_command_action
  behavior_tree_node_lib
  bt_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
