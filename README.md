# KinectStreamer

## Table of Contents
* [**Description**](README.md#description)
* [**Acknowledgements**](README.md#acknowledgements)
* [**Prerequisites**](README.md#prerequisites)
* [**Building with ROS (catkin)**](README.md#building-with-ros-catkin)
* [**Running KinectStreamer**](README.md#running-kinect-streamer)
## Description

Driver for recording and viewing Kinect v2 streams.

**Features**
- **kinect_recorder** -> Recording to disk from Kinect v2 with multiple Kinects
- **kinect_viewer** -> Viewing binary files written by program with multiple Kinects
- See chapter [Running Kinect](README.md#running-kinect-streamer) below for usage

## Acknowledgements
- OpenKinect: libfreenect2
    - https://github.com/OpenKinect/libfreenect2
    - [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.50641.svg)](https://doi.org/10.5281/zenodo.50641)

## Prerequisites
### Dependencies
```console
sudo apt install git build-essential
```
```console
sudo apt-get install libgtk-3-dev
```
```
git clone https://github.com/morrisfranken/argparse.git ~/argparse
```
### Installing OpenCV (Optional)
```console
# Install a whole bunch of prerequisites
sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
```
```console
# Install minimal prerequisites (Ubuntu 18.04 as reference)
sudo apt update && sudo apt install -y cmake g++ wget unzip
```
```console
# Download sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
```
```console
# Unpack sources
unzip opencv.zip
```
```console
# Create build directory
mkdir -p ~/opencv/build && cd ~/opencv/build
```
```console
# Configure
cmake  ../../opencv-master
```
```console
# Build
cmake --build .
```
### Installing libfreenect2
Follow instructions on OpenKinect GitHub
https://github.com/OpenKinect/libfreenect2

### Installing argparse
Follow instructions from morrisfranken:
https://github.com/morrisfranken/argparse
- Essentially, place this respository into your home directory.

## Building with ROS (catkin)\
### If you haven't already got a ROS Catkin Workspace:
```console
mkdir ~/catkin_ws
```
```console
cd ~/catkin_ws
```
```console
mkdir src
```
```console
git clone https://github.com/uqmvale6/kinect_streamer.git src/kinect_streamer
```
```console
catkin_make
```
## Setting Up for Running with ROS
```console
cd ~/catkin_ws
```
```console
source ./devel/setup.sh
```
## Running Kinect Streamer
### Recording to Disk
- WARNING: Recommended disk space - ~20 MB per frame (approx 28 FPS)
- Example Space: 1 hour -> 2 TB (for two Kinects running at 30 FPS)
```console
# Usage
roslaunch kinect_streamer_bin kinect_recorder.launch
```
### Viewing from Disk
```console
# Usage
roslaunch kinect_streamer_bin kinect_viewer.launch
```
### Running Demo PointCloud Software
```console
# Usage
roslaunch kinect_streamer_bin kinect_camera.launch
```
