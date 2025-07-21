add_executable(RT_ScriptExamples 
				RT_ScriptExamples.cpp
				MotionPlanningExamples.cpp)

target_link_libraries(RT_ScriptExamples PUBLIC RT_Script LLSMC)