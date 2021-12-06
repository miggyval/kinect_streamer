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

void KinectDevice::getPointCloud(const float* depth, const uint32_t* registered, uint8_t* cloud_data, int width, int height) {

    float* D = (float*)malloc(DEPTH_W * DEPTH_H * sizeof(float));
    memcpy(D, depth, DEPTH_W * DEPTH_H * sizeof(float));
    uint32_t* R = (uint32_t*)malloc(DEPTH_W * DEPTH_H * sizeof(uint32_t));
    memcpy(R, registered, DEPTH_W * DEPTH_H * sizeof(uint32_t));

    uint8_t* cloud_data_gpu;
    float* X_gpu = NULL;
    float* D_gpu = NULL;
    uint32_t* R_gpu = NULL;

    cudaError_t err = cudaSuccess;
    err = cudaMalloc((void**)&cloud_data_gpu, DEPTH_W * DEPTH_H * sizeof(uint8_t));

    err = cudaMalloc((void**)&D_gpu, DEPTH_W * DEPTH_H * sizeof(float));
    err = cudaMalloc((void**)&R_gpu, DEPTH_W * DEPTH_H * sizeof(uint32_t));

    err = cudaMemcpy(D_gpu, D, DEPTH_W * DEPTH_H * sizeof(float), cudaMemcpyHostToDevice);
    err = cudaMemcpy(R_gpu, R, DEPTH_W * DEPTH_H * sizeof(uint32_t), cudaMemcpyHostToDevice);


    float cx = ir_params.cx;
    float cy = ir_params.cy;
    float fx = 1 / ir_params.fx;
    float fy = 1 / ir_params.fy;

    getPointXYZHelper(D_gpu, R_gpu, cloud_data_gpu, cx, cy, fx, fy, DEPTH_W, DEPTH_H);
    err = cudaFree(&cloud_data_gpu);
    err = cudaFree(D_gpu);
    err = cudaFree(R_gpu);

    err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("%s ", cudaGetErrorString(err));
    }

    free(D);
    free(R);

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