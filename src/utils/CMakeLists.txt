file(GLOB ALGO_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(utils STATIC ${ALGO_SRC})
target_include_directories(utils PUBLIC ${PROJECT_SOURCE_DIR}/include)
