cmake_minimum_required(VERSION 3.0.2)
project(Viewer)

find_package(Pangolin 0.4 REQUIRED)
find_package(OpenCV 4 REQUIRED)


include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(viewer src/viewer.cpp
                      src/PinholeCamera.cpp)

target_link_libraries(viewer ${Pangolin_LIBRARIES} 
                             ${OpenCV_LIBS})