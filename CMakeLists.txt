
cmake_minimum_required(VERSION 3.8)
project(joint_states_parser)
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
 
add_executable(joint_states_parser_node
  src/joint_states_parser_node.cpp
)
 
ament_target_dependencies(joint_states_parser_node
  rclcpp
  sensor_msgs
)
 
install(TARGETS joint_states_parser_node
  DESTINATION lib/${PROJECT_NAME}
)
 
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
 
ament_package()
