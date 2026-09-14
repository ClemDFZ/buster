cmake_minimum_required(VERSION 3.10)
project(tank_viz)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/joint_state_aggregator
  scripts/ghost_publisher
  scripts/viz_markers
  scripts/jpeg_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
