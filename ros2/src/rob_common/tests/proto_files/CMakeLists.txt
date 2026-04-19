cmake_minimum_required(VERSION 3.0.0)

set(proto_root_dir ${CMAKE_CURRENT_SOURCE_DIR}/../)
if(aarch_cc)
  set(Protobuf_INCLUDE_DIR ${protobuf_install_dir}/include)
  set(Protobuf_LIBRARIES ${protobuf_install_dir}/lib/libprotobuf.a)
else()  
  find_package(Protobuf 3 REQUIRED)
endif(aarch_cc)


include(${proto_root_dir}/proto_files/cmake/generate_protobuf.cmake)

set(proto_path ${proto_root_dir}/proto_files)
set(proto_gen_dir ${proto_root_dir}/generated)
generate_proto_cpp_file(${proto_path} ${proto_gen_dir})
include_directories(${proto_gen_dir})
file(GLOB_RECURSE robot_proto_cpp_files
  ${proto_gen_dir}/*.h
  ${proto_gen_dir}/*.cc
)
add_library(robot_proto_lib STATIC
  ${robot_proto_cpp_files}
)
target_link_libraries(robot_proto_lib
  ${Protobuf_LIBRARIES}
)

file(REMOVE ${proto_gen_dir}/CMakeLists.txt)
file(REMOVE_RECURSE ${proto_gen_dir}/cmake)