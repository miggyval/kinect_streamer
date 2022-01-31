cmake_minimum_required( VERSION 3.10 )

project( kinect_streamer_lib )

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(kinect_streamer_lib_VERSION 1.0)

include(GenerateExportHeader)

set( freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

find_package( freenect2 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( CUDA REQUIRED )

find_package( PCL REQUIRED )
find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinect_streamer_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    $ENV{HOME}/argparse/include
    ${PCL_INCLUDE_DIRS}
)


add_executable( kinect_demo	src/kinect_demo.cpp )

cuda_add_library( kinect_depth src/kinect_depth.cu )
add_library( kinect_streamer_lib SHARED src/kinect_streamer.cpp )

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries( kinect_demo ${freenect2_LIBRARIES} ${OpenCV_LIBS} kinect_streamer_lib stdc++fs )
target_link_libraries( kinect_streamer_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES} ${CUDA_LIBRARIES} kinect_depth stdc++fs )
target_link_libraries( kinect_depth ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CUDA_LIBRARIES} )


set_target_properties( kinect_demo	PROPERTIES CXX_STANDARD 17 )

add_compile_options(${GTK3_CFLAGS_OTHER})

install(TARGETS kinect_streamer_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS kinect_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
