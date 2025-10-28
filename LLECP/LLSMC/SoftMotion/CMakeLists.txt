add_subdirectory(motion_algorithm)
add_library(SoftMotion SHARED
    SoftMotion.cpp
    SoftMotionPDO.cpp
    SoftMotionPlanner.cpp
)

target_include_directories(SoftMotion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(SoftMotion PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

target_link_libraries(SoftMotion PUBLIC motion_algorithm)