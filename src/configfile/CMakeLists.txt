file(GLOB COMMON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(configfile STATIC ${COMMON_SRC})

target_include_directories(configfile PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src/utils
)

# 链接 json 
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(configfile
        PRIVATE ${CMAKE_SOURCE_DIR}/lib/JsonCpp/debug/libjsoncpp.a
        utils
    )
else()
    target_link_libraries(configfile
        PRIVATE ${CMAKE_SOURCE_DIR}/lib/JsonCpp/release/libjsoncpp.a
        utils
    )
endif()
