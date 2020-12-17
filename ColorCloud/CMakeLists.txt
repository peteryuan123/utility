cmake_minimum_required(VERSION 2.8.3)
project(coloring_cloud)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -Wall -fopenmp ${CMAKE_CXX_FLAGS}")

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)
find_package(Boost REQUIRED filesystem)

#find_package(Boost REQUIRED COMPONENTS system)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(color_pointcloud ColorCloudVideo.cpp)
add_executable(test_calib ColorImageCloud.cpp)
add_executable(colorXml ColorCloudXML.cpp tinyxml2.cpp)
add_executable(colorMulti ColorCloudMultiImage.cpp)
target_link_libraries(color_pointcloud 
                        ${OpenCV_LIBS}
                        ${PCL_LIBRARIES}
)
target_link_libraries(colorMulti 
                        ${OpenCV_LIBS}
                        ${PCL_LIBRARIES}
                        ${Boost_LIBRARIES}
)
target_link_libraries(test_calib
                        ${OpenCV_LIBS}
                        ${PCL_LIBRARIES}
)
target_link_libraries(colorXml 
                    ${OpenCV_LIBS}
                    ${PCL_LIBRARIES}
                    # ${tinyxml2_LIBS}
)