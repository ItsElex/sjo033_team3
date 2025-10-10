cmake_minimum_required(VERSION 3.8)
project(square_formation)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/square_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
