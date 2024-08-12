cmake_minimum_required(VERSION 3.8)
project(linear_slider_controllers)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Disable boost/shared_ptr in pluginlib while building on rpi4
add_compile_definitions(PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(angles REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)
# find_package(ur_dashboard_msgs REQUIRED)
# find_package(ur_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(linear_slider_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  pluginlib
  # angles
  rcutils
  realtime_tools
  # ur_dashboard_msgs
  # ur_msgs
  # trajectory_msgs
  generate_parameter_library
  controller_manager
  controller_interface
  joint_trajectory_controller
  linear_slider_msgs
  generate_parameter_library
)

include_directories(include)
link_directories(lib)

generate_parameter_library(
  limit_switch_state_broadcaster_parameters
  src/limit_switch_state_broadcaster_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  # src/combined_controller.cpp
  src/limit_switch_state_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  limit_switch_state_broadcaster_parameters
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wpedantic -Werror=return-type)

# target_compile_definitions(${PROEJCT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(
  controller_interface
  controller_plugins.xml
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the plugin XML file
install(
  FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config # launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# install(
#   DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
