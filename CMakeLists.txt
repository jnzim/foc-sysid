cmake_minimum_required(VERSION 3.16)
project(trap_spi)
set(CMAKE_CXX_STANDARD 17)

add_executable(drive
    src/main.cpp
    src/profile.cpp
    src/spi.cpp
    src/chirp.cpp
)
target_include_directories(drive PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(drive PRIVATE lgpio)