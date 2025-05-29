file(GLOB ALGO_SRC 
            ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
            ${PROJECT_SOURCE_DIR}/lib/gram_savitzky_golay/gram_savitzky_golay.cpp
            ${PROJECT_SOURCE_DIR}/lib/gram_savitzky_golay/spatial_filters.cpp
    )

add_library(algorithm STATIC ${ALGO_SRC})
target_include_directories(algorithm PUBLIC 
                            ${PROJECT_SOURCE_DIR}/include
                            ${PROJECT_SOURCE_DIR}/lib
                            ${PROJECT_SOURCE_DIR}/src/utils
                            ${PROJECT_SOURCE_DIR}/lib/gram_savitzky_golay/
                        )
