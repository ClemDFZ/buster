cmake_minimum_required(VERSION 3.10)
project(tank_pantilt)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/pantilt_bridge
  scripts/csi_republish
  scripts/csi_argus
  flash_esp.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config firmware
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
