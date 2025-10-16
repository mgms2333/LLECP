foreach(EXAMPLE 
		RT_ScriptExamples 
		MotionPlanningExamples
    EtherCATMasterExamples
    )
    add_executable(${EXAMPLE} ${EXAMPLE}.cpp)
    target_link_libraries(${EXAMPLE} PUBLIC 
    RT_Script     
    MotionAlgorithm
    CIA402Axis
    FunctionBlock 
    EtherCATMaster
    )
endforeach()
