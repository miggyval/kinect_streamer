cmake_minimum_required( VERSION 3.10 )



project( kinect_streamer )

set( freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2)
set( OpenCV_DIR $ENV{HOME}/opencv/build)

find_package(freenect2 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

include_directories(
    include
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    $ENV{HOME}/argparse/include
)


add_executable( kinect_recorder src/kinect_recorder.cpp )
add_executable( kinect_viewer src/kinect_viewer.cpp )
add_executable( kinect_recorder_gui src/kinect_recorder_gui.c )


target_link_libraries( kinect_recorder ${freenect2_LIBRARIES} ${OpenCV_LIBS} stdc++fs )
target_link_libraries( kinect_viewer ${freenect2_LIBRARIES} ${OpenCV_LIBS} stdc++fs)
target_link_libraries( kinect_recorder_gui ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES} stdc++fs )

set_target_properties( kinect_recorder PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_viewer PROPERTIES CXX_STANDARD 17 )

add_compile_options(${GTK3_CFLAGS_OTHER})