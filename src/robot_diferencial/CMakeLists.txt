cmake_minimum_required(VERSION 3.8)
project(robot_diferencial)

find_package(ament_cmake REQUIRED)

# Carpetas que se copian a install/share/robot_diferencial
install(DIRECTORY urdf worlds launch rviz config docs
  DESTINATION share/${PROJECT_NAME})

ament_package()
