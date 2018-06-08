cmake_minimum_required(VERSION 2.8.3)
project(libviso2)

find_package(catkin REQUIRED )
find_package(OpenCV REQUIRED )

#set(CMAKE_CXX_FLAGS "-msse3")
add_definitions(-msse3)

add_compile_options(-std=c++11)
message( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )
message( STATUS "OUTER OpenCV Include Dir: " ${OpenCV_INCLUDE_DIRS} )

catkin_package(
  INCLUDE_DIRS libviso2/src
  LIBRARIES viso2
)

add_definitions(-Wall -march=native -O3)

include_directories(libviso2/src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(viso2
  libviso2/src/fast.cpp
  libviso2/src/filter.cpp
  libviso2/src/matcher.cpp
  libviso2/src/matrix.cpp
  libviso2/src/triangle.cpp
  libviso2/src/viso.cpp
  libviso2/src/viso_stereo.cpp)

set_target_properties(viso2 PROPERTIES OUTPUT_NAME viso2)


