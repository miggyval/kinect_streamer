#ifndef KINECT_STREAMER_HPP
#define KINECT_STREAMER_HPP

#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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
    libfreenect2::Freenect2Device::ColorCameraParams color_params;
    libfreenect2::Freenect2Device::IrCameraParams ir_params;
    std::string serial;
    libfreenect2::FrameMap frames;
public:
    KinectDevice(std::string serial);
    ~KinectDevice();
    void wait_frames();
    void init_registration();
    void init_params();
    libfreenect2::Frame* get_frame(libfreenect2::Frame::Type type);
    void release_frames();
    int start();
    int stop();
    libfreenect2::Registration* get_registration();
    void getPointCloudCuda(const float* depth, const uint32_t* registered, uint8_t* cloud_data, int width, int height);
    void getPointCloudCpu(const float* depth, const uint32_t* registered, uint8_t* cloud_data, int width, int height);
    void set_color_params(float cx, float cy, float fx, float fy);
    void set_ir_params(float cx, float cy, float fx, float fy, float k1, float k2, float k3, float p1, float p2);
    void get_color_params(float& cx, float& cy, float& fx, float& fy);
    void get_ir_params(float& cx, float& cy, float& fx, float& fy, float k1, float k2, float k3, float p1, float p2);
};
}

#endif