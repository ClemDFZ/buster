cmake_minimum_required(VERSION 3.8)
project(tank_sim_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_web.py
  scripts/rewrite_frame_id.py
  scripts/pantilt_sim_adapter.py
  scripts/sim_nav_contract.py
  scripts/parity_lib.py
  scripts/sim_cleanup.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
