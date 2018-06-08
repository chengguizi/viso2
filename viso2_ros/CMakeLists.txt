### NO.1 CMake Version
cmake_minimum_required(VERSION 3.5)

### NO.2 Project Name
project(viso2_ros)

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
  tf 
  pcl_ros 
  std_srvs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS signals thread)

### NO.4 Python module support

### NO.5 Message / Service / Action Generator
add_message_files( DIRECTORY msg FILES VisoInfo.msg)

### NO.6 Invoke Generation
generate_messages(
  DEPENDENCIES 
    std_msgs
)

### NO.7 Specify package build info export
catkin_package()

### NO.8 libraries / excutables
add_definitions(-Wall -march=native -O3)

include_directories(src ${libviso2_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(stereo_odometer
  src/stereo_odometer.cpp)
add_dependencies(stereo_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(stereo_odometer ${PROJECT_NAME}_gencpp)


#target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


target_link_libraries(stereo_odometer ${Boost_LIBRARIES} ${catkin_LIBRARIES})
