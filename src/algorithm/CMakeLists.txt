file(GLOB ALGO_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(algorithm STATIC ${ALGO_SRC})
target_include_directories(algorithm PUBLIC ${PROJECT_SOURCE_DIR}/include)
