#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <kinect_streamer/kinect_streamer.hpp>

namespace KinectStreamer {

KinectDevice::KinectDevice() {

    pipeline = new libfreenect2::OpenGLPacketPipeline();
    freenect2 = new libfreenect2::Freenect2;
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    if (freenect2->enumerateDevices() == 0) {

    }
    serial = freenect2->getDefaultDeviceSerialNumber();
    dev = freenect2->openDevice(serial, pipeline);
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->startStreams(true, true);

}

void KinectDevice::get_frame() {
    if (!listener->waitForNewFrame(frames, 1000)) {
        std::cout << "Error!" << std::endl;
    }
    libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
}

KinectDevice::~KinectDevice() {
    delete pipeline;
    delete freenect2;
    delete listener;
}

}