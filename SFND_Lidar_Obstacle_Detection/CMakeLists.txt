cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/kdtree3d.h src/render/render.cpp src/processPointClouds.cpp)
add_executable (ransac src/quiz/ransac/ransac2d.cpp src/render/render.cpp src/processPointClouds.cpp)
add_executable (quizCluster src/quiz/cluster/cluster.cpp src/quiz/cluster/kdtree.h src/render/render.cpp)
#add_executable (quizPCABox src/render/pcaBox.cpp src/render/render.cpp)

target_link_libraries (environment ${PCL_LIBRARIES})
target_link_libraries (ransac ${PCL_LIBRARIES})
target_link_libraries (quizCluster ${PCL_LIBRARIES})
#target_link_libraries (quizPCABox ${PCL_LIBRARIES})




