cmake_minimum_required(VERSION 3.10)
project(tank_vlm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(tank_vlm_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/vlm_bridge
  scripts/fetch_models.sh
  scripts/setup_venv.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES models/README.md
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
