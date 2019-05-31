cmake_minimum_required(VERSION 3.5)
project(tf_monitor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/sim_node.py
  scripts/monitor_node.py
  scripts/test_node.py
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
