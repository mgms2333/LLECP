# 添加子模块
add_subdirectory(MotionPlanning)

# 包装成接口库（不编译代码，仅传递链接和头文件）
add_library(MotionAlgorithm INTERFACE)

target_include_directories(MotionAlgorithm INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/MotionPlanning
)

target_link_libraries(MotionAlgorithm INTERFACE MotionPlanning)
