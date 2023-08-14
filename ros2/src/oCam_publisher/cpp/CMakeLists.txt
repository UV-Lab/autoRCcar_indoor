cmake_minimum_required (VERSION 3.10.2)
set(PROJECT_NAME oCam_1CGN_U_T)
project(${PROJECT_NAME} LANGUAGES CXX)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(../api ./api)
        
set(LINK_LIBS
    LIB_OCAM
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
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

add_executable(oCam main.cpp)
target_link_libraries (oCam ${LINK_LIBS})
target_include_directories(oCam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
  )
