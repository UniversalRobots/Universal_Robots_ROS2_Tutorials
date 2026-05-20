cmake_minimum_required(VERSION 3.8)
project(my_robot_cell_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  option(MY_ROBOT_CELL_CONTROL_BUILD_INTEGRATION_TESTS
    "Build integration tests that require URSim and the ur_robot_driver." OFF)

  if(${MY_ROBOT_CELL_CONTROL_BUILD_INTEGRATION_TESTS})
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/example_move.py
      TIMEOUT
        300
    )
  endif()
endif()

ament_package()
