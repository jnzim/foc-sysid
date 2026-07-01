cmake_minimum_required(VERSION 3.16)
project(foc_sysid)
set(CMAKE_CXX_STANDARD 17)

add_executable(drive
    src/main.cpp
    src/protocol.cpp
)

target_include_directories(drive PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(drive PRIVATE lgpio)