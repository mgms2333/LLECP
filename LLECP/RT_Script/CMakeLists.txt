# 创建一个静态库（也可用 SHARED 变动态库）
add_library(RT_Script
    RT_Script.cpp
    RT_ScriptActuator.cpp
    RT_ScriptInterpreter.cpp
    RT_ScriptCalculator.cpp
    RT_ScriptActuatorUint.cpp
    ScriptUint/ScriptUint_Token.cpp
    ScriptUint/ScriptUint_Cmd.cpp
    ScriptUint/ScriptUint_Line.cpp
    ScriptUint/ScriptUint_Script.cpp
)

# 包含头文件目录（提供给主程序使用）
target_include_directories(RT_Script PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
