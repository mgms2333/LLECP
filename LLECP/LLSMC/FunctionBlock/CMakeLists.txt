

add_library(FunctionBlock SHARED
    FunctionBlockBase.cpp
    FB_subclass/FB_Power.cpp
)

target_include_directories(FunctionBlock PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(FunctionBlock PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

target_link_libraries(FunctionBlock PUBLIC CIA402Axis MotionAlgorithm)
