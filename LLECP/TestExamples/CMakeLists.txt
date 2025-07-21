foreach(EXAMPLE 
		RT_ScriptExamples 
		MotionPlanningExamples)
    add_executable(${EXAMPLE} ${EXAMPLE}.cpp)
    target_link_libraries(${EXAMPLE} PUBLIC RT_Script LLSMC)
endforeach()
