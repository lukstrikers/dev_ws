cmake_minimum_required(VERSION 3.5)
project(hand_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
