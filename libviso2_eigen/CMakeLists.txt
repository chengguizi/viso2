cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_COMPILER g++-7)
set(CMAKE_C_COMPILER gcc-7)

project(libviso2_eigen)

# enable C++11 standard (current directory scope)
# set(CMAKE_CXX_STANDARD 17) # cmake 3.5 has not yet support c++17 for this keyword
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-ignored-attributes -Wno-int-in-bool-context -pedantic -std=gnu++17 -fomit-frame-pointer -mavx2 -march=native -mfma -flto -funroll-all-loops -fpeel-loops -ftracer -ftree-vectorize")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

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
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}

  PRIVATE
    include/viso2_eigen
    include/KFAST
)

target_link_libraries(viso2_eigen
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

