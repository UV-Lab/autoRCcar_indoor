cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME ocam_publisher)
project(${PROJECT_NAME} LANGUAGES CXX)

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(api)

set(INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    )

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
  -Wfatal-errors
  )

add_executable(ocam_node ros2/publisher.cpp)
target_link_libraries (ocam_node PRIVATE ${LINK_LIBS})
target_include_directories(ocam_node PRIVATE ${INCLUDE_DIRS})

install(TARGETS ocam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
