cmake_minimum_required( VERSION 3.10 )

project( kinect )

find_package(freenect2 REQUIRED)

include_directories( ${freenect2_INCLUDE_DIR} )

add_executable( kinect_demo src/main.cpp )

target_link_libraries( kinect_demo ${freenect2_LIBRARIES} )