# KinectStreamer

## Table of Contents
* [**Description**](README.md#description)
* [**Acknowledgements**](README.md#acknowledgements)
* [**Prerequisites**](README.md#prerequisites)
* [**Downloading**](README.md#downloading)
* [**Building**](README.md#building)
* [**Running KinectStreamer**](README.md#running-kinect-streamer)
## Description

Driver for recording and viewing Kinect v2 streams.

**Features**
- **kinect_recorder** -> Recording to disk from Kinect v2
- **kinect_viewer** -> Viewing binary files written by program
- See chapter [Running Kinect](README.md#running-kinect) below for usage

## Acknowledgements
- OpenKinect: libfreenect2
    - https://github.com/OpenKinect/libfreenect2
    - [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.50641.svg)](https://doi.org/10.5281/zenodo.50641)

## Prerequisites
### Dependencies
```console
sudo apt-get git build-essential
```
### Installing OpenCV
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
Follow instructions from morrisfranken
https://github.com/morrisfranken/argparse

## Downloading

Through https
```console
git clone https://github.com/uqmvale6/kinect_streamer.git
```
Through ssh
```console
git clone git@github.com:uqmvale6/kinect_streamer.git
```

Through wget
```console
wget -O main.zip https://github.com/uqmvale6/kinect_streamer/archive/refs/heads/main.zip
unzip main.zip
mv kinect_streamer-main kinect_streamer
```
Through zip download
- https://github.com/uqmvale6/kinect_streamer/archive/refs/heads/main.zip

## Building
```console
# Go to kinect_streamer directory
cd kinect_streamer
# (Optional) Set library directories in CMakeLists.txt for alternate installations
set(freenect2_DIR <PATH>)
set(OpenCV_DIR <PATH>)
# Run CMake command
cmake .
# Run Makefile
make -j`nproc`
```
## Running Kinect Streamer
### Recording to Disk
- WARNING: Recommended disk space - 10 MB per frame (approx 28 FPS)
- Example Space: 1 hour -> 1 TB
- Compress after session
```console
# Usage
./kinect_recorder <folder>
# Example
./kinect_recorder ${HOME}/Desktop/kinect_002/patient_x
```
### Viewing from Disk
```console
# Usage
./kinect_viewer <folder>
# Example
./kinect_viewer ${HOME}/Desktop/kinect_002/patient_x
```
