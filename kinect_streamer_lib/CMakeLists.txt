cmake_minimum_required( VERSION 3.10 )

project( kinect_streamer_lib )

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(kinect_streamer_VERSION 1.0)

include(GenerateExportHeader)

set( freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2)
set( OpenCV_DIR $ENV{HOME}/opencv/build)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

find_package( freenect2 REQUIRED )
find_package( OpenCV REQUIRED )

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinect_streamer_lib
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
	include
	${freenect2_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${GTK3_INCLUDE_DIRS}
	$ENV{HOME}/argparse/include
)

add_library( kinect_streamer_lib SHARED src/kinect_streamer.cpp )

target_link_libraries( kinect_streamer_lib ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES} stdc++fs )

add_compile_options(${GTK3_CFLAGS_OTHER})
