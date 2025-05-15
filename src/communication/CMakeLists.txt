file(GLOB COMMON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(communication STATIC ${COMMON_SRC})

target_include_directories(communication PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/cpp_interface
)


# 设置 NRC SDK 路径
set(NRC_SDK "${CMAKE_SOURCE_DIR}/lib/nrc_host" CACHE PATH "Path to NRC SDK root")

set(NRC_ROOT "${CMAKE_SOURCE_DIR}/lib/nrc_host" CACHE PATH "Path to NRC SDK root")
set(OPENVR_DLL "${NRC_ROOT}/nrc_host.dll")
set(OPENVR_LIB "${NRC_ROOT}/libnrc_host.dll.a")

# 链接库目录
target_link_directories(communication
    PRIVATE ${NRC_SDK}
)

# 链接 libnrc_host.dll.a
target_link_libraries(communication
    PRIVATE ${OPENVR_LIB}    
)