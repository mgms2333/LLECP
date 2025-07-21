cmake_minimum_required(VERSION 3.10)
project(LLECP CXX)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(RT_Script)
add_subdirectory(LLSMC)

add_executable(LLECP main.cpp)

target_link_libraries(LLECP PUBLIC RT_Script LLSMC)
