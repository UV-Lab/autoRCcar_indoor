cmake_minimum_required(VERSION 3.16)
# project(ocam)
set(PROJECT_NAME ocam)
project(${PROJECT_NAME} LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(api)

set(LINK_LIBS
    LIB_OCAM
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    )

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release by default" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_compile_options(
    "$<$<CONFIG:Debug>:-O0;-g>"
    "$<$<CONFIG:Release>:-O3>"
    "$<$<CONFIG:None>:-O3>"
    -std=c++17
    -fPIC
    -W
    -Wfatal-errors
    # -Wall
    )

add_executable(ocam_node ros2/publisher.cpp)
target_link_libraries (ocam_node ${LINK_LIBS})
target_include_directories(ocam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ocam_node
  DESTINATION lib/${PROJECT_NAME})

# Install the launch files
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
