cmake_minimum_required(VERSION 3.8)
project(autorccar_hardware_control)

add_compile_options(-O3)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autorccar_interfaces REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

# hardware_control_node
add_executable(hardware_control_node
  src/hardware_control_node.cpp
  src/hardware_control.cpp
)

ament_target_dependencies(hardware_control_node
  rclcpp 
  std_msgs
  autorccar_interfaces
)

target_include_directories(hardware_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

install(TARGETS hardware_control_node
  DESTINATION lib/${PROJECT_NAME})


# launch
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
