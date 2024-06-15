cmake_minimum_required(VERSION 3.22)
project(linear_slider_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
# find_package(rviz REQUIRED)

install(
  DIRECTORY
    meshes
    urdf
    config
    rviz
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()