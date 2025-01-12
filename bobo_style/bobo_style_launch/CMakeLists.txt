cmake_minimum_required(VERSION 3.8)
project(bobo_style_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

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


add_executable(switch_mode src/switch_mode.cpp)
ament_target_dependencies(switch_mode rclcpp std_msgs)



install(TARGETS
  switch_mode
  DESTINATION lib/${PROJECT_NAME})



# # Install Python executables
# install(PROGRAMS

#   scripts/tk.py
#   scripts/switch_mode.py
#   scripts/which_map_server.py
#   # launch/mapping.launch.py
#   DESTINATION lib/${PROJECT_NAME}
# )


ament_package()
