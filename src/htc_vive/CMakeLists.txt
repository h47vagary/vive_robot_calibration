file(GLOB VIVE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(htc_vive STATIC ${VIVE_SRC})
target_include_directories(htc_vive PUBLIC ${PROJECT_SOURCE_DIR}/include)
