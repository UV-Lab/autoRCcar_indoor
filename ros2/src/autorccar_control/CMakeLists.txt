cmake_minimum_required(VERSION 3.8)
project(autorccar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autorccar_interfaces REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pure_pursuit src/main.cpp src/pure_pursuit.cpp src/toolbox.cpp)
add_executable(control_simulation src/main_sim.cpp src/pure_pursuit.cpp src/toolbox.cpp)
add_executable(manual_control src/main_manual.cpp src/pure_pursuit.cpp src/toolbox.cpp)

ament_target_dependencies(pure_pursuit rclcpp std_msgs geometry_msgs autorccar_interfaces)
ament_target_dependencies(control_simulation rclcpp std_msgs geometry_msgs autorccar_interfaces)
ament_target_dependencies(manual_control rclcpp std_msgs geometry_msgs autorccar_interfaces)

install(TARGETS
  pure_pursuit
  control_simulation
  manual_control
  DESTINATION lib/${PROJECT_NAME})


ament_package()
