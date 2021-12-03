#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <kinect_depth/kinect_depth.h>
#include <cuda_runtime.h>
#include <cstdio>
#include <cstring>

#include <kinect_streamer/kinect_streamer.hpp>

namespace KinectStreamer {

KinectDevice::KinectDevice(std::string serial) {

    pipeline = new libfreenect2::CudaPacketPipeline();
    freenect2 = new libfreenect2::Freenect2;
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    if (freenect2->enumerateDevices() == 0) {
        std::cout << "No devices found!" << std::endl;
        throw std::exception();
    }
    if (serial == "") {
        serial = freenect2->getDefaultDeviceSerialNumber();
    }
    dev = freenect2->openDevice(serial, pipeline);
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
}

void KinectDevice::init_registration() {
    color_params = dev->getColorCameraParams();
    ir_params = dev->getIrCameraParams();   
    registration = new libfreenect2::Registration(ir_params, color_params);
}

int KinectDevice::start() {
    return dev->start();
    
}

int KinectDevice::stop() {
    return dev->stop();
}

void KinectDevice::getPointCloud(const float* data, float* X, float* Y, float* Z, int width, int height) {
    int size = DEPTH_W * DEPTH_H * sizeof(float);

    float *D = (float*)malloc(size);
    memcpy(D, data, size);
    float *X_gpu = NULL;
    float *Y_gpu = NULL;
    float *Z_gpu = NULL;
    float *D_gpu = NULL;

    cudaError_t err = cudaSuccess;
    err = cudaMalloc((void**)&X_gpu, size);
    err = cudaMalloc((void**)&Y_gpu, size);
    err = cudaMalloc((void**)&Z_gpu, size);
    err = cudaMalloc((void**)&D_gpu, size);

    err = cudaMemcpy(D_gpu, D, size, cudaMemcpyHostToDevice);

    float cx = ir_params.cx;
    float cy = ir_params.cy;
    float fx = 1 / ir_params.fx;
    float fy = 1 / ir_params.fy;

    getPointXYZHelper(D_gpu, X_gpu, Y_gpu, Z_gpu, cx, cy, fx, fy, DEPTH_W, DEPTH_H);

    err = cudaMemcpy(X, X_gpu, size, cudaMemcpyDeviceToHost);
    err = cudaMemcpy(Y, Y_gpu, size, cudaMemcpyDeviceToHost);
    err = cudaMemcpy(Z, Z_gpu, size, cudaMemcpyDeviceToHost);
    
    err = cudaFree(X_gpu);
    err = cudaFree(Y_gpu);
    err = cudaFree(Z_gpu);
    err = cudaFree(D_gpu);

    err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("%s\n\r", cudaGetErrorString(err));
    }
    free(D);

}

void KinectDevice::wait_frames() {
    if (!listener->waitForNewFrame(frames, 10 * 1000)) {
        std::cout << "Error!" << std::endl;
        throw std::exception();
    }
}
libfreenect2::Frame* KinectDevice::get_frame(libfreenect2::Frame::Type type) {
    return frames[type];
}

void KinectDevice::release_frames() {
    listener->release(frames);
}

KinectDevice::~KinectDevice() {
    delete pipeline;
    delete freenect2;
    delete listener;
    delete registration;
}

libfreenect2::Registration* KinectDevice::get_registration() {
    return registration;
}

}