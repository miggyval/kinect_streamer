cmake_minimum_required( VERSION 3.10 )

project( kinect_streamer_ros )

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(kinect_streamer_ros_VERSION 1.0)

include(GenerateExportHeader)

set( freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

find_package( freenect2 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( camera_info_manager REQUIRED )

find_package( PCL REQUIRED )
find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
	pcl_ros
	std_msgs
	sensor_msgs
	image_transport
	cv_bridge
	camera_info_manager
	pcl_conversions
)

find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinect_streamer_ros
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
	${freenect2_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_library( kinect_streamer_ros SHARED src/kinect_ros.cpp )

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries( kinect_streamer_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS} kinect_depth stdc++fs )

install(TARGETS kinect_streamer_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


add_compile_options(${GTK3_CFLAGS_OTHER})