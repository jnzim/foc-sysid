cmake_minimum_required(VERSION 3.16)
project(MotionController)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/motion
)

file(GLOB_RECURSE SOURCES
    motion/*.cpp
    src/*.cpp
)

add_executable(MotionControl ${SOURCES})

target_link_libraries(MotionControl m)