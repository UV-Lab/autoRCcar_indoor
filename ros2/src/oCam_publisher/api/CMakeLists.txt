find_library(UDEV_LIB udev)
find_library(V4L2_LIB v4l2)

message("-- Found LIB_UDEV ${UDEV_LIB}")
message("-- Found LIB_V4L2 ${V4L2_LIB}")

set(LINK_LIBS
    ${UDEV_LIB}
    ${V4L2_LIB}
    )

add_library(LIB_OCAM STATIC
    withrobot_camera.cpp
    withrobot_utility.cpp
    )

target_include_directories (LIB_OCAM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/api)
target_link_libraries (LIB_OCAM ${LINK_LIBS})