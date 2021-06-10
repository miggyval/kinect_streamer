cmake_minimum_required( VERSION 3.10 )

project( kinect_streamer_project )

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

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${freenect2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    $ENV{HOME}/argparse/include
)

add_library( kinect_streamer SHARED ${CMAKE_SOURCE_DIR}/src/kinect_streamer.cpp )

target_link_libraries( kinect_streamer ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES} stdc++fs )

add_compile_options(${GTK3_CFLAGS_OTHER})