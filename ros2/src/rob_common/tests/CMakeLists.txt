cmake_minimum_required(VERSION 3.5)

########### test ###########
# set(test_dir ${CMAKE_CURRENT_SOURCE_DIR})
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

########### rob_bridge test 
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto_files ${PROJECT_BINARY_DIR}/proto_generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/generated)

# The example of topic publishing.
add_executable(topic_pub_test
    ${test_dir}/topic_pub_main.cpp
)
target_link_libraries(topic_pub_test
    robot_proto_lib
    ${PROJECT_NAME}
)

# The example of topic subscription.
add_executable(topic_sub_test
    ${test_dir}/topic_sub_main.cpp
)
target_link_libraries(topic_sub_test
    robot_proto_lib
    ${PROJECT_NAME}
)

# The example of robot server.
add_executable(robot_server_test 
    ${test_dir}/robot_server_test.cpp
)
target_link_libraries(robot_server_test
    robot_proto_lib
    ${PROJECT_NAME}
)

# The example of robot client.
add_executable(robot_client_test 
    ${test_dir}/robot_client_test.cpp
)
target_link_libraries(robot_client_test
    robot_proto_lib
    ${PROJECT_NAME}
)

# The test of ShM client and server.
add_executable(robot_shm_server_test
    ${test_dir}/robot_shm_server_test.cpp
)
target_link_libraries(robot_shm_server_test
    robot_proto_lib
    ${PROJECT_NAME}
)

add_executable(robot_shm_client_test
    ${test_dir}/robot_shm_client_test.cpp
)
target_link_libraries(robot_shm_client_test
    robot_proto_lib
    ${PROJECT_NAME}
)