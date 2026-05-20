cmake_minimum_required(VERSION 3.8)
project(my_dual_robot_cell_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/example_move.py
    TIMEOUT
      300
  )
endif()

ament_package()
