cmake_minimum_required(VERSION 3.0.0)

add_subdirectory(${CMAKE_SOURCE_DIR}/third_parties third_parties_binary_dir)

set(third_parties_dir ${CMAKE_CURRENT_SOURCE_DIR}/../third_parties)
include_directories(${third_parties_dir}/json11/include)
# include_directories(${third_parties_dir}/nng/include)
include_directories(${third_parties_dir}/nanomsg/include)
include_directories(${third_parties_dir}/spdlog/include)
include_directories(${third_parties_dir}/yaml-cpp/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## add the lib: mc_com_lib
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/log_config)
set(mc_com_dir ${CMAKE_CURRENT_SOURCE_DIR}/log_config)
file(GLOB_RECURSE mc_com_src_files
    ${mc_com_dir}/*.h
    ${mc_com_dir}/*.cpp 
)
add_library(mc_com_lib STATIC ${mc_com_src_files})
target_link_libraries(mc_com_lib
    spdlog::spdlog
    yaml-cpp::yaml-cpp
    json_11_lib
)

## add the robot_bridge_lib
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/robot_bridge)
set(robot_bridge_dir ${CMAKE_CURRENT_SOURCE_DIR}/robot_bridge/robot_bridge)
file(GLOB_RECURSE robot_bridge_src_files
    ${robot_bridge_dir}/*.h
    ${robot_bridge_dir}/*.cpp
    ${robot_bridge_dir}/*.inl
)
add_library(robot_bridge_lib STATIC
    ${robot_bridge_src_files}
)
target_link_libraries(robot_bridge_lib
    # ${nng_lib}
    # nng
    nanomsg
)