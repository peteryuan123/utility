cmake_minimum_required(VERSION 2.8)

project(video_to_image)
find_package(Boost REQUIRED filesystem)
find_package(OpenCV REQUIRED)

FIND_PACKAGE( OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(video_to_image video_to_image.cpp)
target_link_libraries(video_to_image ${Boost_LIBRARIES} ${OpenCV_LIBS})