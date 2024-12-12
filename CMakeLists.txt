cmake_minimum_required(VERSION 3.8)
project(manymove_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)

# Generate messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveManipulator.action"
  "msg/MovementConfig.msg"
  DEPENDENCIES geometry_msgs action_msgs
)

# Get the typesupport target for linking
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

# Create the shared library
add_library(manymove_planner_lib
  src/manymove_planner.cpp
)

ament_target_dependencies(manymove_planner_lib
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
  rosidl_default_runtime  # Included rosidl runtime dependencies
)

# Link the generated typesupport library
target_link_libraries(manymove_planner_lib ${cpp_typesupport_target})

install(TARGETS
  manymove_planner_lib
  DESTINATION lib
)

####################
# Main example node
####################
add_executable(manymove_planner_node src/main.cpp)
target_link_libraries(manymove_planner_node manymove_planner_lib)
ament_target_dependencies(manymove_planner_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
)
# Link the generated typesupport library if needed
target_link_libraries(manymove_planner_node ${cpp_typesupport_target})

install(TARGETS manymove_planner_node
  DESTINATION lib/${PROJECT_NAME}
)


###########################################
# Move Action Server (Single Action)
###########################################
add_executable(action_server_node
  src/action_server_node.cpp
  src/action_server.cpp
)
target_link_libraries(action_server_node manymove_planner_lib)
ament_target_dependencies(action_server_node
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
  action_msgs
  rosidl_default_runtime  # Ensure rosidl runtime is included
)
target_link_libraries(action_server_node ${cpp_typesupport_target})

install(TARGETS action_server_node
  DESTINATION lib/${PROJECT_NAME}
)


############################
# Move Client Example
############################
add_executable(client_example_node src/client_example_node.cpp)
ament_target_dependencies(client_example_node
  rclcpp
  rclcpp_action
  geometry_msgs
  action_msgs
  rosidl_default_runtime  # Ensure rosidl runtime is included
)
target_link_libraries(client_example_node ${cpp_typesupport_target})

install(TARGETS client_example_node
  DESTINATION lib/${PROJECT_NAME}
)


#########################
# Install config & launch
#########################
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

#########################
# Exporting
#########################
ament_export_libraries(manymove_planner_lib)
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_visual_tools
  action_msgs
  rclcpp_action
  rosidl_default_runtime
)
ament_export_include_directories(include)

ament_package()
