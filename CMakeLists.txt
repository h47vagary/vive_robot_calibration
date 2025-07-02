cmake_minimum_required(VERSION 3.10)
project(vive_robot_calibration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt 所需
set(CMAKE_PREFIX_PATH "D:/QT/5.12.9/mingw73_64/lib/cmake")
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets PrintSupport)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 头文件路径
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/configfile
    ${PROJECT_SOURCE_DIR}/src/htc_vive
    ${PROJECT_SOURCE_DIR}/src/algorithm
    ${PROJECT_SOURCE_DIR}/src/utils
    ${PROJECT_SOURCE_DIR}/src/communication
    ${PROJECT_SOURCE_DIR}/lib/openvr/headers
    ${PROJECT_SOURCE_DIR}/lib/jsoncpp/include
    ${PROJECT_SOURCE_DIR}/lib/eigen
)

# 添加子模块（编译为静态库）
add_subdirectory(src/algorithm)
add_subdirectory(src/communication)
add_subdirectory(src/htc_vive)
add_subdirectory(src/utils)
add_subdirectory(src/configfile)
add_subdirectory(src/ui)


# 添加主程序
add_executable(vive_robot_calibration src/main.cpp)

# 链接各模块静态库
target_link_libraries(vive_robot_calibration
    PRIVATE
        algorithm
        communication
        htc_vive
        utils
        configfile
        ui
)

if (WIN32)
    target_link_libraries(vive_robot_calibration PRIVATE winmm)
endif()

# OpenVR SDK 路径
set(OPENVR_ROOT "${PROJECT_SOURCE_DIR}/lib/openvr")
set(OPENVR_BIN_DIR "${OPENVR_ROOT}/bin/win64")
set(OPENVR_DLL "${OPENVR_BIN_DIR}/openvr_api.dll")

# NRC SDK 路径
set(NRC_ROOT "${PROJECT_SOURCE_DIR}/lib/nrc_host")
set(NRC_DLL "${NRC_ROOT}/nrc_host.dll")

# 编译完成后将 openvr_api.dll 和 nrc_host.dll 复制到可执行文件目录
add_custom_command(TARGET vive_robot_calibration POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${OPENVR_DLL}
        ${NRC_DLL}
        $<TARGET_FILE_DIR:vive_robot_calibration>
)
