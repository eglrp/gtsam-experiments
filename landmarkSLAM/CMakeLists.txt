cmake_minimum_required(VERSION 2.6)
project(simpleLocalization C CXX)

set(GTSAM_DIR "/home/km/libs/gtsam-3.2.1/build")
find_package(GTSAM)
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${GTSAM_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

add_executable(landmarkSLAM landmarkSLAM.cpp)
target_link_libraries(landmarkSLAM gtsam)
