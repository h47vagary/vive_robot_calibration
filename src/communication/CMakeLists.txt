file(GLOB COMMON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(communication STATIC ${COMMON_SRC})

target_include_directories(communication PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/cpp_interface
        ${PROJECT_SOURCE_DIR}/src/utils
)


# 设置 NRC SDK 路径
set(NRC_SDK "${CMAKE_SOURCE_DIR}/lib/nrc_host" CACHE PATH "Path to NRC SDK root")

set(NRC_ROOT "${CMAKE_SOURCE_DIR}/lib/nrc_host" CACHE PATH "Path to NRC SDK root")
set(NRC_DLL "${NRC_ROOT}/nrc_host.dll")
set(NRC_LIB "${NRC_ROOT}/libnrc_host.dll.a")

# 链接库目录
target_link_directories(communication
    PRIVATE ${NRC_SDK}
)


# 链接 libnrc_host.dll.a
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(communication
        PRIVATE ${CMAKE_SOURCE_DIR}/lib/JsonCpp/debug/libjsoncpp.a
        ${NRC_LIB}
    )
else()
    target_link_libraries(communication
        PRIVATE ${CMAKE_SOURCE_DIR}/lib/JsonCpp/release/libjsoncpp.a
        ${NRC_LIB}
    )
endif()

target_link_libraries(communication
    PUBLIC
        Qt5::Widgets
        utils
)