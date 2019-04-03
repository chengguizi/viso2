### NO.1 CMake Version
cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_COMPILER g++-7)
set(CMAKE_C_COMPILER gcc-7)

### NO.2 Project Name
project(viso2_ros)

# enable C++11 standard (current directory scope)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-ignored-attributes -Wno-int-in-bool-context -pedantic -std=gnu++17 -fomit-frame-pointer -mavx2 -march=native -mfma -flto -funroll-all-loops -fpeel-loops -ftracer -ftree-vectorize")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

### NO.3 Fine other CMake / Catkin Packages needed for build
find_package(catkin REQUIRED COMPONENTS 
  libviso2_eigen 
  roscpp 
  sensor_msgs 
  nav_msgs 
  message_filters 
  image_transport 
  cv_bridge 
  image_geometry 
  tf2_ros
  tf2_eigen
  rosbag
  std_srvs
  message_generation

  dlib_ros
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)

### NO.5 Message / Service / Action Generator
add_message_files( DIRECTORY msg FILES VisoInfo.msg)

### NO.6 Invoke Generation
generate_messages(
  DEPENDENCIES 
    std_msgs
)

### NO.7 Specify package build info export
# This function must be called before declaring any targets with add_library() or add_executable().
catkin_package(
  CATKIN_DEPENDS libviso2_eigen
)

#include_directories(include ${catkin_INCLUDE_DIRS})

### NO.8 libraries / excutables

add_executable(stereo_odometer
  src/stereo_odometer.cpp
  src/ros_publisher.cpp)

target_include_directories( stereo_odometer
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# add_library(ros_publisher
#   src/ros_publisher.cpp)

#   target_include_directories( ros_publisher
#   PUBLIC
#     ${catkin_INCLUDE_DIRS}
# )

# message("viso2_eigen_LIBRARIES=${viso2_eigen_LIBRARIES}")
# message("libviso2_eigen_LIBRARIES=${libviso2_eigen_LIBRARIES}")
# message("catkin_LIBRARIES=${catkin_LIBRARIES}")

target_link_libraries(stereo_odometer ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# target_link_libraries(ros_publisher ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(stereo_odometer ${PROJECT_NAME}_generate_messages_cpp)
