cmake_minimum_required(VERSION 3.8)
project(cr3_custom_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()