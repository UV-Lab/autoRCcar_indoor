cmake_minimum_required(VERSION 3.5)
project(rob_common)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

link_libraries(stdc++fs)

# RelWithDebInfo, Debug Release
set(CMAKE_BUILD_TYPE Release)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rmw REQUIRED)
find_package(spdlog_vendor)

# Add the iceoryx lib.
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)


# find the package of nav_common.
message(STATUS "---------------------------------------------")
message(STATUS "CMake 路径调试信息：")
message(STATUS "CMAKE_SOURCE_DIR         = ${CMAKE_SOURCE_DIR}")

add_subdirectory(${CMAKE_SOURCE_DIR}/third_parties third_parties_binary_dir)
set(third_parties_dir ${CMAKE_CURRENT_SOURCE_DIR}/third_parties)
include_directories(${third_parties_dir}/json11/include)
# include_directories(${third_parties_dir}/nng/include)
include_directories(${third_parties_dir}/nanomsg/include)
# include_directories(${third_parties_dir}/spdlog/include)
include_directories(${third_parties_dir}/yaml-cpp/include)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rob_common
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rob_common/log_config
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rob_common/log_config/config_wraper
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rob_common/robot_bridge
)

## add the lib: mc_com_lib
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}src/log_config)
# set(rob_com_dir ${CMAKE_CURRENT_SOURCE_DIR}/src/log_config)
set(rob_com_dir ${CMAKE_CURRENT_SOURCE_DIR}/src)
file(GLOB_RECURSE rob_com_src_files
    ${rob_com_dir}/*.h
    ${rob_com_dir}/*.cpp 
)
add_library(${PROJECT_NAME} STATIC ${rob_com_src_files})
# target_include_directories(nav_com_lib PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp spdlog_vendor
)
# target_link_libraries(${PROJECT_NAME}
# )
target_link_libraries(${PROJECT_NAME} PUBLIC
    spdlog::spdlog
    yaml-cpp::yaml-cpp
    json_11_lib
    nanomsg
    iceoryx_posh::iceoryx_posh iceoryx_hoofs::iceoryx_hoofs
)
# target_include_directories(${PROJECT_NAME} PRIVATE ${iceoryx_hoofs_INCLUDE_DIRS})

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY third_parties/json11/include/
  DESTINATION include
)
install(
  DIRECTORY third_parties/yaml-cpp/include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# install(TARGETS json_11_lib 
#   EXPORT ${PROJECT_NAME} 
#   DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )
install(TARGETS json_11_lib 
  EXPORT json_11_lib
  DESTINATION lib/json_11_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
# install(TARGETS spdlog 
#   EXPORT ${PROJECT_NAME} 
#   DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )
install(TARGETS yaml-cpp
  EXPORT ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# ## add the robot_bridge_lib
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/robot_bridge)
# set(robot_bridge_dir ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_bridge)
# file(GLOB_RECURSE robot_bridge_src_files
#     ${robot_bridge_dir}/*.h
#     ${robot_bridge_dir}/*.cpp
#     ${robot_bridge_dir}/*.inl
# )
# add_library(robot_bridge_lib STATIC
#     ${robot_bridge_src_files}
# )
# target_link_libraries(robot_bridge_lib
#     nanomsg
# )


########### test ###########
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)
# set(test_dir ${CMAKE_CURRENT_SOURCE_DIR}/tests)
# file(COPY ${test_dir}/resources DESTINATION ${CMAKE_BINARY_DIR}/tests/)
# # The example of config and log tools.
# add_executable(log_test 
#     ${test_dir}/log_test.cpp
# )
# target_link_libraries(log_test
#   ${PROJECT_NAME}
# )
# ament_target_dependencies(log_test
#   rclcpp spdlog_vendor 
# )

# add_executable(config_test 
#     ${test_dir}/config_test.cpp
# )
# target_link_libraries(config_test
#   ${PROJECT_NAME}
# )
# ament_target_dependencies(config_test
#   rclcpp spdlog_vendor 
# )


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

ament_package()
