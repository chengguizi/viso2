cmake_minimum_required(VERSION 3.5)
project(libviso2)

find_package(catkin REQUIRED )
find_package(OpenCV REQUIRED )

#set(CMAKE_CXX_FLAGS "-msse3")
add_compile_options(-msse3)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  INCLUDE_DIRS libviso2/src
  LIBRARIES libviso2
)

add_library(libviso2
  libviso2/src/fast.cpp
  libviso2/src/filter.cpp
  libviso2/src/matcher.cpp
  libviso2/src/matrix.cpp
  libviso2/src/triangle.cpp
  libviso2/src/viso.cpp
  libviso2/src/viso_stereo.cpp)

target_include_directories(libviso2
  PUBLIC
    libviso2/src
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(libviso2
  ${OpenCV_LIBRARIES}
)



