cmake_minimum_required(VERSION 3.10)
project(LLECP CXX)


set(CMAKE_CXX_STANDARD 17)

include_directories(/opt/etherlab/include)
link_directories(/opt/etherlab/lib)

add_subdirectory(EtherCATMaster)
add_subdirectory(RT_Script)
add_subdirectory(LLSMC)
add_subdirectory(TestExamples)
add_executable(LLECP main.cpp)
target_link_libraries(LLECP PUBLIC RT_Script MotionAlgorithm CIA402Axis FunctionBlock pthread 
EtherCATMaster
)


target_include_directories(LLECP PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/MotionAlgorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/CIA402Axis
    ${CMAKE_CURRENT_SOURCE_DIR}/FunctionBlock
)