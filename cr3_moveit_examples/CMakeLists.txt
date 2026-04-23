cmake_minimum_required(VERSION 3.8)
project(cr3_moveit_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(cr3_first_program src/cr3_first_program.cpp)
ament_target_dependencies(
  cr3_first_program
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(cr3_box_sorter_moveit src/cr3_box_sorter_moveit.cpp)
ament_target_dependencies(
  cr3_box_sorter_moveit
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  cr3_first_program
  cr3_box_sorter_moveit
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()