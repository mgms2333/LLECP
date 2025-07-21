# 收集 LLSMC 的源文件
set(SMC_SOURCES
    LL_SMC.cpp
)

add_subdirectory(MotionAlgorithm)
# add_subdirectory(MotionFunctionBlock)  # 如启用，再解注释

add_library(LLSMC STATIC ${SMC_SOURCES})

target_include_directories(LLSMC PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/MotionAlgorithm
    # ${CMAKE_CURRENT_SOURCE_DIR}/MotionFunctionBlock
)

target_link_libraries(LLSMC PUBLIC
    MotionAlgorithm
    # MotionFunctionBlock
)
