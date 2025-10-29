# hrsmc/CIA402Axis/CMakeLists.txt
add_library(CIA402Axis SHARED
    CIA402Axis.cpp
    CIA402AxisPDO.cpp
    CIA402AxisConfig.cpp
    CIA402AxisGeneral.cpp
)

target_include_directories(CIA402Axis PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(CIA402Axis PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
