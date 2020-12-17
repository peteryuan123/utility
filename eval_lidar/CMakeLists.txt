cmake_minimum_required(VERSION 2.8.3)
project(coloring_cloud)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -Wall -fopenmp ${CMAKE_CXX_FLAGS}")

find_package(PCL REQUIRED)

find_package(Boost REQUIRED filesystem)

#find_package(Boost REQUIRED COMPONENTS system)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(eval eval.cpp)
add_executable(eval2 eval2.cpp)

target_link_libraries(eval 
                    ${PCL_LIBRARIES})

target_link_libraries(eval2 
    ${PCL_LIBRARIES})
