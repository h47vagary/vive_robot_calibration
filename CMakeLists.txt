cmake_minimum_required(VERSION 3.10)
project(vive_robot_calibration)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件路径
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/lib/openvr/headers
    ${PROJECT_SOURCE_DIR}/lib/jsoncpp/include
    ${PROJECT_SOURCE_DIR}/lib/eigen
)

# 添加子模块（编译为静态库）
add_subdirectory(src/algorithm)
add_subdirectory(src/communication)
add_subdirectory(src/htc_vive)
add_subdirectory(src/utils)

# 添加主程序
add_executable(vive_robot_calibration src/main.cpp)

# 链接各模块静态库
target_link_libraries(vive_robot_calibration
    algorithm
    communication
    htc_vive
    utils
)
