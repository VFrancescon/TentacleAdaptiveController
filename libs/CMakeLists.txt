cmake_minimum_required(VERSION 3.1.0)
project(AdaptiveControllerLibs)

add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


include_directories(include)
add_subdirectory(src)