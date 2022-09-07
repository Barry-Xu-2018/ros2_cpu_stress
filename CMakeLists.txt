cmake_minimum_required(VERSION 3.8)
project(stress)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(talker_component SHARED src/publisher_member_function.cpp)
ament_target_dependencies(talker_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(talker_component
  PLUGIN "StressPublisher"
  EXECUTABLE talker
)

add_library(listener_component SHARED src/subscriber_member_function.cpp)
ament_target_dependencies(listener_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(listener_component
  PLUGIN "StressSubscriber"
  EXECUTABLE listener
)

install(TARGETS
  talker_component
  listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
