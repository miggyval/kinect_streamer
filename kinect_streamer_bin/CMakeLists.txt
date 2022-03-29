cmake_minimum_required( VERSION 3.10 )

project( kinect_streamer_bin )

set( freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2 )

find_package( freenect2 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( sensor_msgs REQUIRED )
find_package( cv_bridge REQUIRED )
find_package( PkgConfig REQUIRED )
find_package( camera_info_manager REQUIRED )
find_package( CUDA REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  kinect_streamer_lib
  kinect_streamer_ros
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
  pcl_conversions
)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)


catkin_package(
  LIBRARIES kinect_streamer_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${kinect_streamer_lib_INCLUDE_DIRS}
    ${kinect_streamer_ros_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    $ENV{HOME}/argparse/include
)

link_directories(
    ${kinect_streamer_lib_LIB_DIR}
    ${kinect_streamer_ros_LIB_DIR}
)

# Add Executable Files
add_executable( kinect_camera	src/nodes/kinect_camera.cpp )

add_executable( kinect_recorder	src/nodes/kinect_recorder.cpp )
add_executable( kinect_viewer	src/nodes/kinect_viewer.cpp )

add_executable( kinect_recorder_gui	src/nodes/kinect_recorder_gui.cpp )
add_executable( kinect_viewer_gui	src/nodes/kinect_viewer_gui.cpp )

# Link Libraries
target_link_libraries( kinect_camera ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CUDA_LIBRARIES} kinect_streamer_ros kinect_streamer_lib stdc++fs )

target_link_libraries( kinect_recorder	${freenect2_LIBRARIES} ${OpenCV_LIBS} kinect_streamer_lib stdc++fs )
target_link_libraries( kinect_viewer	${freenect2_LIBRARIES} ${OpenCV_LIBS} kinect_streamer_lib stdc++fs)

target_link_libraries( kinect_recorder_gui	${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES} kinect_streamer_lib stdc++fs )
target_link_libraries( kinect_viewer_gui	${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES} kinect_streamer_lib stdc++fs )


# Set to C++17
set_target_properties( kinect_camera	PROPERTIES CXX_STANDARD 17 )

set_target_properties( kinect_recorder	PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_viewer	PROPERTIES CXX_STANDARD 17 )

set_target_properties( kinect_recorder_gui	PROPERTIES CXX_STANDARD 17 )
set_target_properties( kinect_viewer_gui	PROPERTIES CXX_STANDARD 17 )
	
add_compile_options(${GTK3_CFLAGS_OTHER}) 
