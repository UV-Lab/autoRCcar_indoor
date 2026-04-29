cmake_minimum_required(VERSION 3.10)
project(pcl_converter)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------
# 关键配置：尝试静态链接
# -----------------------------------------------------------

# 1. 全局设置：优先查找静态库
# 这会告诉 CMake 在 find_package 时优先寻找 .a 文件而不是 .so 文件
set(BUILD_SHARED_LIBS ON)

# 2. 查找 PCL 包
# 注意：Ubuntu 默认安装的 libpcl-dev 主要是动态库。
# 如果要完全静态编译，你可能需要手动编译 PCL 源码并安装。
# 这里我们尝试查找，如果系统只有动态库，CMake 可能会回退使用动态库，
# 或者报错（取决于具体库的提供情况）。
find_package(PCL REQUIRED COMPONENTS common io)

# -----------------------------------------------------------
# 构建可执行文件
# -----------------------------------------------------------

# 添加可执行文件
add_executable(${PROJECT_NAME} main.cpp)

# 包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    ${PCL_INCLUDE_DIRS}
)

# 链接库
# PCL_LIBRARIES 变量包含了所有必要的库路径
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
)

# 定义编译选项（可选）
# 防止一些 Windows 风格的宏定义报错
add_definitions(${PCL_DEFINITIONS})

# -----------------------------------------------------------
# 调试信息输出
# -----------------------------------------------------------
message(STATUS "PCL found at: ${PCL_DIR}")
message(STATUS "PCL Libraries to link: ${PCL_LIBRARIES}")