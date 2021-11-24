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
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
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

int KinectDevice::start() {
    return dev->start();
}

int KinectDevice::stop() {
    return dev->stop();
}

libfreenect2::Frame* KinectDevice::get_frame(libfreenect2::Frame::Type type) {
    if (!listener->waitForNewFrame(frames, 200)) {
        std::cout << "Error!" << std::endl;
        throw std::exception();
    }
    return frames[type];
}

void KinectDevice::release_frames() {
    return listener->release(frames);
}

KinectDevice::~KinectDevice() {
    delete pipeline;
    delete freenect2;
    delete listener;
}

}