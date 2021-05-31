cmake_minimum_required( VERSION 3.10 )



project( kinect_streamer )

find_package(freenect2 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(
    ${include}
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    $ENV{HOME}/argparse/include
)


add_executable( kinect_recorder src/kinect_recorder.cpp )
add_executable( kinect_viewer src/kinect_viewer.cpp )

target_link_libraries( kinect_recorder ${freenect2_LIBRARIES} ${OpenCV_LIBS} stdc++fs )
target_link_libraries( kinect_viewer ${freenect2_LIBRARIES} ${OpenCV_LIBS} stdc++fs)

set_target_properties( kinect_recorder PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_viewer PROPERTIES CXX_STANDARD 17 )

