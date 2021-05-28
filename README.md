# KinectRecorder
## Prerequisites
### Dependencies
```console
sudo apt-get git build-essential
```
### Installing OpenCV
```
# Install a whole bunch of prerequisites
sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
# Install minimal prerequisites (Ubuntu 18.04 as reference)
sudo apt update && sudo apt install -y cmake g++ wget unzip
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
unzip opencv.zip
# Create build directory
mkdir -p build && cd build
# Configure
cmake  ../opencv-master
# Build
cmake --build .
```
### Installing libfreenect2
Follow instructions on OpenKinect GitHub
https://github.com/OpenKinect/libfreenect2

## Downloading
```console
git clone {repo}
```
## Building
```console
# Go to kinect directory
cd kinect
# Run CMake command
cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2 -DOpenCV_DIR=$HOME/opencv/build .
# Run Makefile
make -j`nproc`
```
## Running Kinect
```console
# Usage
./kinect_write <folder>
# Example
./kinect_write $HOME/Desktop/kinect_002/patient_x
```
