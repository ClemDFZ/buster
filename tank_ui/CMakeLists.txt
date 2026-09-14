cmake_minimum_required(VERSION 3.10)
project(tank_ui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/ui_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
