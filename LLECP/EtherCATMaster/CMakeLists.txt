# 创建一个静态库（也可用 SHARED 变动态库）
add_library(EtherCATMaster
    #EtherCATMaster.cpp
)

# 包含头文件目录（提供给主程序使用）
target_include_directories(EtherCATMaster PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
