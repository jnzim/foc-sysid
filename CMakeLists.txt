cmake_minimum_required(VERSION 3.16)
project(trap_spi)

set(CMAKE_CXX_STANDARD 17)


add_executable(MotionControl
    src/main.cpp
    src/profile.cpp
    src/spi.cpp
)

target_include_directories(MotionControl PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)