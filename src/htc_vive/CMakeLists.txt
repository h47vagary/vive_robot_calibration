# 搜索当前目录下所有 cpp 文件作为源文件
file(GLOB VIVE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# 设置 OpenVR SDK 路径
set(OPENVR_ROOT "${CMAKE_SOURCE_DIR}/lib/openvr" CACHE PATH "Path to OpenVR SDK root")
set(OPENVR_INCLUDE_DIR "${OPENVR_ROOT}/headers")
set(OPENVR_LIB_DIR "${OPENVR_ROOT}/lib/win64")
set(OPENVR_BIN_DIR "${OPENVR_ROOT}/bin/win64")
set(OPENVR_DLL "${OPENVR_BIN_DIR}/openvr_api.dll")
set(OPENVR_LIB "${OPENVR_LIB_DIR}/openvr_api.lib")

# 构建静态库
add_library(htc_vive STATIC ${VIVE_SRC})

# 包含头文件目录
target_include_directories(htc_vive
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
    PRIVATE
        ${OPENVR_INCLUDE_DIR}
)

# 链接 openvr 静态库
target_link_libraries(htc_vive PRIVATE
                        ${OPENVR_LIB}
                        utils                         
)