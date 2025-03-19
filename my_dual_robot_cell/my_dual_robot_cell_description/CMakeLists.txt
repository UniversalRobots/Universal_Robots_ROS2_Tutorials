cmake_minimum_required(VERSION 3.5)
project(my_dual_robot_cell_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
