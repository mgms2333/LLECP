foreach(EXAMPLE 
		RT_ScriptExamples 
		MotionPlanningExamples
    EtherCATMasterExamples
    )
    add_executable(${EXAMPLE} ${EXAMPLE}.cpp)
    target_link_libraries(${EXAMPLE} PUBLIC 
    RT_Script     
    CIA402Axis
    EtherCATMaster
    motion_algorithm
    )
endforeach()
