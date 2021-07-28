cmake_minimum_required( VERSION 3.10 )

project( kinect_streamer_bin )

set( freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2 )
set( OpenCV_DIR ${CMAKE_SOURCE_DIR}/opencv/build )

find_package( freenect2 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( PkgConfig REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  kinect_streamer_lib
  std_msgs
  roscpp
  rospy
)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)


include_directories(
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${kinect_streamer_lib_INCLUDE_DIRS}
    $ENV{HOME}/argparse/include
)

link_directories(
    ${kinect_streamer_LIB_DIR}
)

add_executable( kinect_recorder_cli	src/nodes/kinect_recorder_cli.cpp )
add_executable( kinect_viewer_cli	src/nodes/kinect_viewer_cli.cpp )
add_executable( kinect_recorder_gui	src/nodes/kinect_recorder_gui.cpp )
add_executable( kinect_viewer_gui	src/nodes/kinect_viewer_gui.cpp )

target_link_libraries( kinect_recorder_cli	${freenect2_LIBRARIES} ${OpenCV_LIBS} ${kinect_streamer_LIBS} stdc++fs )
target_link_libraries( kinect_viewer_cli	${freenect2_LIBRARIES} ${OpenCV_LIBS} ${kinect_streamer_LIBS} stdc++fs)

target_link_libraries( kinect_recorder_gui	${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES}  ${kinect_streamer_LIBS} stdc++fs )
target_link_libraries( kinect_viewer_gui	${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES}  ${kinect_streamer_LIBS} stdc++fs )

set_target_properties( kinect_recorder_cli	PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_recorder_gui	PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_viewer_cli	PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_viewer_gui	PROPERTIES CXX_STANDARD 17 )
	
add_compile_options(${GTK3_CFLAGS_OTHER}) 
