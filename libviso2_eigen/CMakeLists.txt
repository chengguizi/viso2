cmake_minimum_required(VERSION 3.5)
project(libviso2_eigen)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES viso2_eigen
)

add_library(viso2_eigen 
  viso2_eigen.cpp
  # quad_matcher.cpp
  stereo_motion_estimator.cpp)

target_include_directories(viso2_eigen
  PUBLIC
    include/viso2_eigen
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(viso2_eigen
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)



