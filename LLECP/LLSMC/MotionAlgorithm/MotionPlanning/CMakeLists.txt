# 收集源文件
file(GLOB_RECURSE MP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# 创建静态库
add_library(MotionPlanning STATIC ${MP_SOURCES})

# 公开头文件路径
target_include_directories(MotionPlanning PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
