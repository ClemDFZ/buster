cmake_minimum_required(VERSION 3.10)
project(tank_track)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(tank_track_interfaces REQUIRED)
find_package(tank_vlm_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/track_server
  scripts/fetch_ostrack.sh
  scripts/setup_venv.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES models/ostrack/README.md
  DESTINATION share/${PROJECT_NAME}/models/ostrack
)

ament_package()
