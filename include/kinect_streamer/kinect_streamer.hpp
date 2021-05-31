#ifndef KINECT_STREAMER_HPP
#define KINECT_STREAMER_HPP

#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

class KinectDevice {
private:
    libfreenect2::Freenect2* freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::PacketPipeline *pipeline;
public:
    KinectDevice();
};

#endif