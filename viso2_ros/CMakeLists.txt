### NO.1 CMake Version
cmake_minimum_required(VERSION 3.5)

### NO.2 Project Name
project(viso2_ros)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

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

  dutils_profiler
)

find_package(OpenCV 4 REQUIRED)
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
# catkin_package(
#   CATKIN_DEPENDS libviso2_eigen
# )

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

target_link_libraries(stereo_odometer ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

# target_link_libraries(ros_publisher ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(stereo_odometer ${PROJECT_NAME}_generate_messages_cpp)
