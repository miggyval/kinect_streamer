#ifndef KINECT_ROS_HPP
#define KINECT_ROS_HPP


#define COLOR_W 1920
#define COLOR_H 1080
#define DEPTH_W 512
#define DEPTH_H 424
#define POINT_STEP 32

#include <signal.h>
#include <cstdio>
#include <cstring>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class KinectROS {
private:
    std::map<std::string, std::unique_ptr<ros::NodeHandle>> nh_camera_info_colors;
    std::map<std::string, std::unique_ptr<ros::NodeHandle>> nh_camera_info_irs;
    std::map<std::string, std::unique_ptr<ros::NodeHandle>> nh_camera_colors;
    std::map<std::string, std::unique_ptr<ros::NodeHandle>> nh_camera_irs;
    std::map<std::string, std::unique_ptr<ros::NodeHandle>> nh_clouds;

    std::map<std::string, std::unique_ptr<camera_info_manager::CameraInfoManager>> manager_colors;
    std::map<std::string, std::unique_ptr<camera_info_manager::CameraInfoManager>> manager_irs;
    
    std::map<std::string, sensor_msgs::CameraInfo> info_colors;
    std::map<std::string, sensor_msgs::CameraInfo> info_irs;

    std::map<std::string, std::unique_ptr<image_transport::ImageTransport>> it_colors;
    std::map<std::string, std::unique_ptr<image_transport::ImageTransport>> it_irs;

    std::map<std::string, ros::Publisher>               pub_clouds;
    std::map<std::string, image_transport::Publisher>   pub_colors;
    std::map<std::string, image_transport::Publisher>   pub_irs;
    std::map<std::string, ros::Publisher>               pub_camera_info_colors;
    std::map<std::string, ros::Publisher>               pub_camera_info_irs;

public:
    KinectROS(int argc, char** argv);
    void init_camera_pub(std::vector<std::string> serials);
    void init_frame_pub(std::vector<std::string> serials);
    void init_cloud_pub(std::vector<std::string> serials);
    void send_image(std::string serial, cv::Mat img_color, cv::Mat img_ir_mono8);
    void send_cloud(std::string serial, uint8_t* cloud_data, int size);
};

#endif