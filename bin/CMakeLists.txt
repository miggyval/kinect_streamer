cmake_minimum_required( VERSION 3.10 )

project( kinect_streamer )

set( freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2 )
set( OpenCV_DIR $ENV{HOME}/opencv/build )
set ( kinect_streamer_INCLUDE_DIR $ENV{HOME}/kinect_streamer/include )
set ( kinect_streamer_LIB_DIR $ENV{HOME}/kinect_streamer/lib )
set ( kinect_streamer_LIBS libkinect_streamer.so )

find_package( freenect2 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( PkgConfig REQUIRED )

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${kinect_streamer_INCLUDE_DIR}
    $ENV{HOME}/argparse/include
)

link_directories(
    ${kinect_streamer_LIB_DIR}
)

add_executable( kinect_recorder ${CMAKE_SOURCE_DIR}/src/kinect_recorder.cpp )
add_executable( kinect_viewer ${CMAKE_SOURCE_DIR}/src/kinect_viewer.cpp )
add_executable( kinect_recorder_gui ${CMAKE_SOURCE_DIR}/src/kinect_recorder_gui.cpp )
add_executable( kinect_viewer_gui ${CMAKE_SOURCE_DIR}/src/kinect_viewer_gui.cpp )
add_executable( kinect_test ${CMAKE_SOURCE_DIR}/src/kinect_test.cpp)


target_link_libraries( kinect_recorder ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${kinect_streamer_LIBS} stdc++fs )
target_link_libraries( kinect_viewer ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${kinect_streamer_LIBS} stdc++fs)

target_link_libraries( kinect_recorder_gui ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES}  ${kinect_streamer_LIBS} stdc++fs )
target_link_libraries( kinect_viewer_gui ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES}  ${kinect_streamer_LIBS} stdc++fs )
target_link_libraries( kinect_test ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES} ${kinect_streamer_LIBS} stdc++fs )

set_target_properties( kinect_recorder PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_viewer PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_test PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_recorder_gui PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_viewer_gui PROPERTIES CXX_STANDARD 17 )

add_compile_options(${GTK3_CFLAGS_OTHER})