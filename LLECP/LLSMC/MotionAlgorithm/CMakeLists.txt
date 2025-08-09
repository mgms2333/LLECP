# 自动收集 MotionPlanning 及其下所有 cpp 文件
file(GLOB_RECURSE MA_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library(MotionAlgorithm SHARED ${MA_SOURCES})

target_include_directories(MotionAlgorithm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/MotionPlanning
)

