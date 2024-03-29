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
# add_compile_definitions(PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(controller_manager REQUIRED)
# find_package(joint_trajectory_controller REQUIRED)
# find_package(velocity_controllers REQUIRED)
# find_package(lifecycle_msgs REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(pluginlib REQUIRED)

# add_executable(linear_slider_control_node src/linear_slider_control_node.cpp)

# ament_target_dependencies(
#   # linear_slider_control_node
#   # rclcpp
#   # controller_manager

# )

# install(TARGETS
#   # linear_slider_control_node
#   DESTINATION lib/${PROJECT_NAME}
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
