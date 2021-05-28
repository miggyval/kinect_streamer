# KinectRecorder
## Prerequisites
### Dependencies
```console
sudo apt-get git build-essential
```
### Installing libfreenect2
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
