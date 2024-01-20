cmake_minimum_required(VERSION 3.26)
project(GlobalPlanner)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_executable(GlobalPlanner
        include/PSO.h
        source/PSO.cpp
        include/Point.h
        include/spline.h
        main.cpp
#        ReadFile.cpp
)
