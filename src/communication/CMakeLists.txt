file(GLOB COMMON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(communication STATIC ${COMMON_SRC})
target_include_directories(communication PUBLIC ${PROJECT_SOURCE_DIR}/include)
