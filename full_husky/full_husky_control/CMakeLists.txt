cmake_minimum_required(VERSION 3.8)
project(full_husky_control)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(static_state_publisher src/static_state_publisher.cpp)
ament_target_dependencies(static_state_publisher rclcpp sensor_msgs)
install(TARGETS
 static_state_publisher
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
