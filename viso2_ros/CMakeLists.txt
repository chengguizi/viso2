### NO.1 CMake Version
cmake_minimum_required(VERSION 3.5)

### NO.2 Project Name
project(viso2_ros)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

### NO.3 Fine other CMake / Catkin Packages needed for build
find_package(catkin REQUIRED COMPONENTS 
  libviso2 
  roscpp 
  sensor_msgs 
  nav_msgs 
  message_filters 
  image_transport 
  cv_bridge 
  image_geometry 
  tf2_ros
  tf2_geometry_msgs
  pcl_ros 
  std_srvs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)

### NO.5 Message / Service / Action Generator
add_message_files( DIRECTORY msg FILES VisoInfo.msg)

### NO.6 Invoke Generation
generate_messages(
  DEPENDENCIES 
    std_msgs
)

### NO.7 Specify package build info export
catkin_package(
  CATKIN_DEPENDS libviso2
)

#include_directories(include ${catkin_INCLUDE_DIRS})

### NO.8 libraries / excutables

add_executable(stereo_odometer
  src/stereo_odometer.cpp)

target_include_directories( stereo_odometer
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(stereo_odometer ${Boost_LIBRARIES} ${catkin_LIBRARIES})
