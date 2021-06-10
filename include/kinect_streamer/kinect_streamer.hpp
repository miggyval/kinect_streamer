#ifndef KINECT_STREAMER_HPP
#define KINECT_STREAMER_HPP

#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>


#define COLOR_W 1920
#define COLOR_H 1080
#define DEPTH_W 512
#define DEPTH_H 424

#define DEPTH_MAX 2000.0
#define FPS_MAX 30

namespace KinectStreamer {


class KinectDevice {
private:
    libfreenect2::Freenect2* freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::PacketPipeline *pipeline;
    libfreenect2::Registration* registration;
    libfreenect2::SyncMultiFrameListener* listener;
    std::string serial;
    libfreenect2::FrameMap frames;
public:
    KinectDevice();
    ~KinectDevice();
    void get_frame();
};
}

#endif