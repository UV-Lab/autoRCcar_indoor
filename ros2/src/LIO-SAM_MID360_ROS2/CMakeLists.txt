cmake_minimum_required(VERSION 3.5)
project(lio_sam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

cmake_policy(SET CMP0074 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(autorccar_interfaces REQUIRED)

include_directories(
  include/lio_sam
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)



add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL livox_ros_driver2)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM Eigen autorccar_interfaces livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam "${cpp_typesupport_target}")

add_executable(${PROJECT_NAME}_mapOptimization src/mapOptmization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM livox_ros_driver2)
if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}" OpenMP::OpenMP_CXX)
else()
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}")
endif()


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_imageProjection
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_imuPreintegration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_featureExtraction
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

set(grid_map_dir "src/grid_map")
file(GLOB_RECURSE grid_map_srcs 
  ${grid_map_dir}/*.cpp
  ${grid_map_dir}/*.h)


add_executable(transform_global_map
  src/TransformGlobalMap.cpp
  ${grid_map_srcs}
)
ament_target_dependencies(transform_global_map sensor_msgs Eigen OpenCV PCL livox_ros_driver2)
target_link_libraries(transform_global_map stdc++fs)
install(
  TARGETS transform_global_map
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(grid_map_builder
  src/GridMapBuilder.cpp
  ${grid_map_srcs}
)
ament_target_dependencies(grid_map_builder sensor_msgs rclcpp nav_msgs pcl_conversions pcl_msgs  visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs livox_ros_driver2 Eigen OpenCV PCL)
target_link_libraries(grid_map_builder stdc++fs)
install(
  TARGETS grid_map_builder
  DESTINATION lib/${PROJECT_NAME}
)


# --- 3.  ---
# find the package of nav_common.
message(STATUS "---------------------------------------------")
message(STATUS "CMake 路径调试信息：")
message(STATUS "PROJECT_SOURCE_DIR       = ${PROJECT_SOURCE_DIR}")
message(STATUS "CMAKE_SOURCE_DIR         = ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR         = ${CMAKE_BINARY_DIR}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(rob_common REQUIRED PATHS ${PROJECT_SOURCE_DIR}/../../install/rob_common/share/rob_common/cmake)
include_directories(${rob_common_INCLUDE_DIRS})
set(rob_common_lib_dir ${rob_common_DIR}/../../../lib)

message(STATUS "rob_common_lib_dir = ${rob_common_lib_dir}")
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)
set(rob_com_libs
  ${rob_common_lib_dir}/libjson_11_lib.a  
  ${rob_common_lib_dir}/libnanomsg.so  
  ${rob_common_lib_dir}/libnanomsg.so.5  
  ${rob_common_lib_dir}/libnanomsg.so.5.1.0  
  ${rob_common_lib_dir}/librob_common.a  
  # ${rob_common_lib_dir}/libspdlog.a  
  spdlog::spdlog stdc++fs
  ${rob_common_lib_dir}/libyaml-cpp.a
  iceoryx_posh::iceoryx_posh iceoryx_hoofs::iceoryx_hoofs
)
link_libraries(stdc++fs)




# msf_loc lib
include_directories(${PROJECT_SOURCE_DIR}/src/scan_context)
set(msf_loc_dir ${PROJECT_SOURCE_DIR}/src/scan_context)
file(GLOB_RECURSE msf_loc_files
  ${msf_loc_dir}/*.cpp
  ${msf_loc_dir}/*.h
)
add_library(msf_loc_lib SHARED ${msf_loc_files})
target_link_libraries(msf_loc_lib 
${rob_com_libs} 
${PCL_LIBRARIES} 
${OpenCV_LIBS}
)
install(TARGETS
  msf_loc_lib
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(slam_map_post_processing
  src/SlamMapPostProcessingMain.cpp
  src/map_splitter/MapSplitter.cpp
)
target_link_libraries(slam_map_post_processing
  ${PCL_LIBRARIES}
  ${rob_com_libs}
  msf_loc_lib
)
install(
  TARGETS slam_map_post_processing
  DESTINATION lib/${PROJECT_NAME}
)

message(STATUS "---------------------------------------------")

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
