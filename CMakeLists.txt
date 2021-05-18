cmake_minimum_required( VERSION 3.10 )

project( kinect )

find_package(freenect2 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable( kinect_write src/kinect_write.cpp )
add_executable( kinect_read src/kinect_read.cpp )

target_link_libraries( kinect_write ${freenect2_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries( kinect_read ${freenect2_LIBRARIES} ${OpenCV_LIBS} )