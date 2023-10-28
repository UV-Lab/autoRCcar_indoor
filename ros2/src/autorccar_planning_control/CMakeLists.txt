cmake_minimum_required(VERSION 3.8)
project(autorccar_planning_control)

add_compile_options(-O3)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(autorccar_interfaces REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

# planning_control_node
add_executable(planning_control_node
  src/planning_control_node.cpp
  src/planning_control.cpp
  src/cubic_spline_path.cpp
)

ament_target_dependencies(planning_control_node
  rclcpp 
  Eigen3
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  autorccar_interfaces
)

target_include_directories(planning_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

install(TARGETS planning_control_node
  DESTINATION lib/${PROJECT_NAME})


# simulator_node
add_executable(simulator_node
  src/simulator_node.cpp
  src/simulator.cpp
)

ament_target_dependencies(simulator_node
  rclcpp 
  std_msgs
  geometry_msgs
  autorccar_interfaces
  Eigen3
)

install(TARGETS simulator_node
  DESTINATION lib/${PROJECT_NAME})


# launch
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
