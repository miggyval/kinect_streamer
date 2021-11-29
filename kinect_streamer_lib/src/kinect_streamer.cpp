#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <kinect_streamer/kinect_streamer.hpp>

namespace KinectStreamer {

KinectDevice::KinectDevice(std::string serial) {

    pipeline = new libfreenect2::OpenGLPacketPipeline();
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

    libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();
    libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
    ir_params.cx = 254.878f;
    ir_params.cy = 205.395f;
    ir_params.fx = 365.456f;
    ir_params.fy = 365.456f;
    ir_params.k1 = 0.0905474;
    ir_params.k2 = -0.26819;
    ir_params.k3 = 0.0950862;
    ir_params.p1 = 0.0;
    ir_params.p2 = 0.0;

    color_params.cx = 970.81224f;
    color_params.cy = 561.04021f;
    color_params.fx = 1062.60584f;
    color_params.fy = 1062.2804f;
    
    dev->setColorCameraParams(color_params);
    dev->setIrCameraParams(ir_params);
    registration = new libfreenect2::Registration(ir_params, color_params);
}

int KinectDevice::start() {
    return dev->start();
}

int KinectDevice::stop() {
    return dev->stop();
}

void KinectDevice::wait_frames() {
    if (!listener->waitForNewFrame(frames, 200)) {
        std::cout << "Error!" << std::endl;
        throw std::exception();
    }
}
libfreenect2::Frame* KinectDevice::get_frame(libfreenect2::Frame::Type type) {
    return frames[type];
}

void KinectDevice::release_frames() {
    return listener->release(frames);
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