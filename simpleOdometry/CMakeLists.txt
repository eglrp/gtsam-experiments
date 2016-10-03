cmake_minimum_required(VERSION 2.6)
project(simpleOdometry C CXX)

set(GTSAM_DIR "/home/km/libs/gtsam-3.2.1/build")
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${GTSAM_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})

add_executable(odometryExample odometryExample.cpp)
target_link_libraries(odometryExample gtsam)

add_executable(odometryExample_symbolic odometryExample_symbolic.cpp)
target_link_libraries(odometryExample_symbolic gtsam)
