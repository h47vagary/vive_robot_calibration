# 包含 QT 文件
set(UI_FORMS
    main_window.ui
)

set(UI_SOURCES
    main_window.cpp    
)

set(UI_HEADERS
    main_window.h    
)

# 自动处理 .ui 文件，生成头文件
qt5_wrap_ui(UI_GENERATED_HEADERS ${UI_FORMS})

# 创建静态库
add_library(ui STATIC ${UI_SOURCES}
                      ${UI_HEADERS}
                      ${UI_GENERATED_HEADERS})

# 包含生成的头文件目录
target_include_directories(ui PUBLIC 
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_BINARY_DIR}
            ${PROJECT_SOURCE_DIR}/include
            ${PROJECT_SOURCE_DIR}/src/communication
            ${PROJECT_SOURCE_DIR}/src/algorithm
            ${PROJECT_SOURCE_DIR}/src/htc_vive
            ${PROJECT_SOURCE_DIR}/src/utils
)

target_link_libraries(ui
    PUBLIC
        Qt5::Widgets
        communication
        htc_vive
)
