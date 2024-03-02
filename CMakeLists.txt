cmake_minimum_required(VERSION 3.26)
project(GlobalPlanner)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_executable(GlobalPlanner
        include/ABCPlanner.h
        include/BasePlanner.h
        include/CostMapLoader.h
        include/Debug.h
        include/GWOPlanner.h
        include/Point.h
        include/PSOPlanner.h
        include/spline.h
        include/Utility.h
        source/main.cpp
)
