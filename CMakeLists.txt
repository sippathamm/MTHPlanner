cmake_minimum_required(VERSION 3.26)
project(MTHPlanner)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_executable(GlobalPlanner
        include/ABCPlanner.h
        include/BasePlanner.h
        include/CostMapLoader.h
        include/IGWOPlanner.h
        include/IPSOPlanner.h
        include/Point.h
        include/Print.h
        include/spline.h
        include/Utility.h
        source/Planner.cpp
)
