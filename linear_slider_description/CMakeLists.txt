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
    # rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
