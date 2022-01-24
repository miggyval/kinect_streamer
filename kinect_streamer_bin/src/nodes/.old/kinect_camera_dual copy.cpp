#include <ros/ros.h>
#include <kinect_streamer/kinect_streamer.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <camera_info_manager/camera_info_manager.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

#include <signal.h>
#include <cstdio>
#include <cstring>

#define myUINT8 1
#define myFLOAT32 7


#define MAX_DEPTH 5000.0

bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s) {
  protonect_shutdown = true;
}

// roslaunch aruco_detect aruco_detect.launch camera:=/color image:=/image_raw dictionary:=1 fiducial_len:=0.120 publish_images:=true
// roslaunch aruco_detect aruco_detect.launch camera:=/color_b image:=/image_raw dictionary:=1 fiducial_len:=0.120 publish_images:=true

int main(int argc, char** argv) {
    libfreenect2::setGlobalLogger(NULL);
    std::string serial_a = "501530742442";
    std::string serial_b = "226287140347";
    //std::string serial_a = "097377233947";
    //std::string serial_b = "220183535047";
    ros::init(argc, argv, "kinect_camera");
    bool show = false;
    bool info = true;
    KinectStreamer::KinectDevice kin_dev_a(serial_a);
    KinectStreamer::KinectDevice kin_dev_b(serial_b);
    if (!kin_dev_a.start()) {
        std::cout << "Failed to start Kinect A." << std::endl;
        exit(-1);
    }
    if (!kin_dev_b.start()) {
        std::cout << "Failed to start Kinect B." << std::endl;
        exit(-1);
    }
    ros::NodeHandle nh_camera_info_color_a;
    ros::NodeHandle nh_camera_info_color_b;

    ros::NodeHandle nh_camera_info_ir_a;
    ros::NodeHandle nh_camera_info_ir_b;

    /* Two clouds! */
    ros::NodeHandle nh_cloud_a;
    ros::NodeHandle nh_cloud_b;
    
    ros::NodeHandle nh_camera_color_a("color_a");
    ros::NodeHandle nh_camera_color_b("color_b");

    ros::NodeHandle nh_camera_ir_a("ir_a");
    ros::NodeHandle nh_camera_ir_b("ir_b");

    camera_info_manager::CameraInfoManager manager_color_a(nh_camera_color_a, "color_a");
    camera_info_manager::CameraInfoManager manager_color_b(nh_camera_color_b, "color_b");

    camera_info_manager::CameraInfoManager manager_ir_a(nh_camera_ir_a, "ir_a");
    camera_info_manager::CameraInfoManager manager_ir_b(nh_camera_ir_b, "ir_b");
    
    ros::Publisher pub_camera_info_color_a = nh_camera_info_color_a.advertise<sensor_msgs::CameraInfo>("color_a/camera_info", 1);
    ros::Publisher pub_camera_info_color_b = nh_camera_info_color_b.advertise<sensor_msgs::CameraInfo>("color_b/camera_info", 1);

    ros::Publisher pub_camera_info_ir_a = nh_camera_info_ir_a.advertise<sensor_msgs::CameraInfo>("ir_a/camera_info", 1);
    ros::Publisher pub_camera_info_ir_b = nh_camera_info_ir_b.advertise<sensor_msgs::CameraInfo>("ir_b/camera_info", 1);

    if (!manager_color_a.loadCameraInfo("")) {
        std::cout << "Failed to get calibration from Color A.yaml" << std::endl;
    }
    if (!manager_color_b.loadCameraInfo("")) {
        std::cout << "Failed to get calibration from Color B.yaml" << std::endl;
    }
    if (!manager_ir_a.loadCameraInfo("")) {
        std::cout << "Failed to get calibration from IR A.yaml" << std::endl;
    }
    if (!manager_ir_b.loadCameraInfo("")) {
        std::cout << "Failed to get calibration from IR B .yaml" << std::endl;
    }

    kin_dev_a.init_params();
    kin_dev_b.init_params();

    sensor_msgs::CameraInfo info_color_a = manager_color_a.getCameraInfo();
    sensor_msgs::CameraInfo info_color_b = manager_color_b.getCameraInfo();
    float cx_color_a, cy_color_a, fx_color_a, fy_color_a;
    float cx_color_b, cy_color_b, fx_color_b, fy_color_b;
    kin_dev_a.get_color_params(cx_color_a, cy_color_a, fx_color_a, fy_color_a);
    kin_dev_b.get_color_params(cx_color_b, cy_color_b, fx_color_b, fy_color_b);
    info_color_a.K[2] = cx_color_a;
    info_color_a.K[5] = cy_color_a;
    info_color_a.K[0] = fx_color_a;
    info_color_a.K[4] = fy_color_a;
    info_color_b.K[2] = cx_color_b;
    info_color_b.K[5] = cy_color_b;
    info_color_b.K[0] = fx_color_b;
    info_color_b.K[4] = fy_color_b;
    manager_color_a.setCameraInfo(info_color_a);
    manager_color_b.setCameraInfo(info_color_b);

    
    sensor_msgs::CameraInfo info_ir_a = manager_ir_a.getCameraInfo();
    sensor_msgs::CameraInfo info_ir_b = manager_ir_b.getCameraInfo();
    float cx_ir_a, cy_ir_a, fx_ir_a, fy_ir_a, k1_ir_a, k2_ir_a, k3_ir_a, p1_ir_a, p2_ir_a;
    float cx_ir_b, cy_ir_b, fx_ir_b, fy_ir_b, k1_ir_b, k2_ir_b, k3_ir_b, p1_ir_b, p2_ir_b;
    kin_dev_a.get_ir_params(cx_ir_a, cy_ir_a, fx_ir_a, fy_ir_a, k1_ir_a, k2_ir_a, k3_ir_a, p1_ir_a, p2_ir_a);
    kin_dev_b.get_ir_params(cx_ir_b, cy_ir_b, fx_ir_b, fy_ir_b, k1_ir_b, k2_ir_b, k3_ir_b, p1_ir_b, p2_ir_b);
    info_ir_a.K[2] = cx_ir_a;
    info_ir_a.K[5] = cy_ir_a;
    info_ir_a.K[0] = fx_ir_a;
    info_ir_a.K[4] = fy_ir_a;
    info_ir_a.D[0] = p1_ir_a;
    info_ir_a.D[1] = p2_ir_a;
    info_ir_a.D[2] = k1_ir_a;
    info_ir_a.D[3] = k2_ir_a;
    info_ir_a.D[4] = k3_ir_a;
    manager_ir_a.setCameraInfo(info_ir_a);
    manager_ir_b.setCameraInfo(info_ir_b);
    
    kin_dev_a.init_registration();
    kin_dev_b.init_registration();
    
    ros::NodeHandle nh_color_a;
    ros::NodeHandle nh_color_b;
    ros::NodeHandle nh_ir_a;
    ros::NodeHandle nh_ir_b;

    image_transport::ImageTransport it_color_a(nh_color_a);
    image_transport::ImageTransport it_color_b(nh_color_b);

    image_transport::ImageTransport it_ir_a(nh_ir_a);
    image_transport::ImageTransport it_ir_b(nh_ir_b);

    image_transport::Publisher pub_color_a = it_color_a.advertise("color_a/image_raw", 1);
    image_transport::Publisher pub_color_b = it_color_b.advertise("color_b/image_raw", 1);

    image_transport::Publisher pub_ir_a = it_ir_a.advertise("ir_a/image_raw", 1);
    image_transport::Publisher pub_ir_b = it_ir_b.advertise("ir_b/image_raw", 1);

    ros::Publisher pub_cloud_a = nh_cloud_a.advertise<sensor_msgs::PointCloud2>("/points_a", 1);
    ros::Publisher pub_cloud_b = nh_cloud_b.advertise<sensor_msgs::PointCloud2>("/points_b", 1);

    while (ros::ok() && !protonect_shutdown) {
        
        kin_dev_a.wait_frames();
        kin_dev_b.wait_frames();

        libfreenect2::Frame* color_a = kin_dev_a.get_frame(libfreenect2::Frame::Color);
        libfreenect2::Frame* depth_a = kin_dev_a.get_frame(libfreenect2::Frame::Depth);
        libfreenect2::Frame* ir_a = kin_dev_a.get_frame(libfreenect2::Frame::Ir);

        libfreenect2::Frame* color_b = kin_dev_b.get_frame(libfreenect2::Frame::Color);
        libfreenect2::Frame* depth_b = kin_dev_b.get_frame(libfreenect2::Frame::Depth);
        libfreenect2::Frame* ir_b = kin_dev_b.get_frame(libfreenect2::Frame::Ir);

        // Create Image from Color Frame
        cv::Mat img_color_a(cv::Size(color_a->width, color_a->height), CV_8UC4, color_a->data);
        cv::Mat img_color_b(cv::Size(color_b->width, color_b->height), CV_8UC4, color_b->data);
        // Create Image from Depth Frame
        cv::Mat img_depth_a(cv::Size(depth_a->width, depth_a->height), CV_32FC1, depth_a->data);
        cv::Mat img_depth_b(cv::Size(depth_b->width, depth_b->height), CV_32FC1, depth_b->data);
        // Create Image from IR Frame
        cv::Mat img_ir_a(cv::Size(ir_a->width, ir_a->height), CV_32FC1, ir_a->data);
        cv::Mat img_ir_b(cv::Size(ir_b->width, ir_b->height), CV_32FC1, ir_b->data);
        img_ir_a /= 8.0;
        img_ir_b /= 8.0;
        cv::Mat img_ir_a_mono8;
        cv::Mat img_ir_b_mono8;
        img_ir_a.convertTo(img_ir_a_mono8, CV_8UC1);
        img_ir_b.convertTo(img_ir_b_mono8, CV_8UC1);

        cv::Mat img_ir_a_bilat;
        cv::Mat img_ir_b_bilat;
        cv::bilateralFilter(img_ir_a_mono8, img_ir_a_bilat, 0, 0.0, 5.0);
        cv::bilateralFilter(img_ir_b_mono8, img_ir_b_bilat, 0, 0.0, 5.0);

        cv::medianBlur(img_depth_a, img_depth_a, 5);
        cv::medianBlur(img_depth_b, img_depth_b, 5);
        
        cv::flip(img_color_a, img_color_a, 1);
        cv::flip(img_color_b, img_color_b, 1);

        cv::flip(img_ir_a_bilat, img_ir_a_bilat, 1);
        cv::flip(img_ir_b_bilat, img_ir_b_bilat, 1);

        // Send Image
        sensor_msgs::ImagePtr msg_color_a = cv_bridge::CvImage(std_msgs::Header(), "bgra8", img_color_a).toImageMsg();
        sensor_msgs::ImagePtr msg_color_b = cv_bridge::CvImage(std_msgs::Header(), "bgra8", img_color_b).toImageMsg();
        sensor_msgs::ImagePtr msg_ir_a = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_ir_a_bilat).toImageMsg();
        sensor_msgs::ImagePtr msg_ir_b = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_ir_b_bilat).toImageMsg();
    
        pub_color_a.publish(msg_color_a);
        pub_color_b.publish(msg_color_b);
        
        pub_ir_a.publish(msg_ir_a);
        pub_ir_b.publish(msg_ir_b);
        // Send Camera Info
        sensor_msgs::CameraInfo info_camera_color_a = manager_color_a.getCameraInfo();
        sensor_msgs::CameraInfo info_camera_color_b = manager_color_b.getCameraInfo();
        sensor_msgs::CameraInfo info_camera_ir_a = manager_ir_a.getCameraInfo();
        sensor_msgs::CameraInfo info_camera_ir_b = manager_ir_b.getCameraInfo();

        info_camera_color_a.header.stamp = ros::Time::now();
        info_camera_color_b.header.stamp = ros::Time::now();

        pub_camera_info_color_a.publish(info_camera_color_a);
        pub_camera_info_color_b.publish(info_camera_color_b);

        info_camera_ir_a.header.stamp = ros::Time::now();
        info_camera_ir_b.header.stamp = ros::Time::now();

        pub_camera_info_ir_a.publish(info_camera_ir_a);
        pub_camera_info_ir_b.publish(info_camera_ir_b);

        // Flip the Image Back
        cv::flip(img_color_a, img_color_a, 1);
        cv::flip(img_color_b, img_color_b, 1);

        libfreenect2::Frame undistorted_a(512, 424, 4);
        libfreenect2::Frame undistorted_b(512, 424, 4);

        libfreenect2::Frame registered_a(512, 424, 4);
        libfreenect2::Frame registered_b(512, 424, 4);
        

        libfreenect2::Registration* registration_a = kin_dev_a.get_registration();
        libfreenect2::Registration* registration_b = kin_dev_b.get_registration();
        
        registration_a->apply(color_a, depth_a, &undistorted_a, &registered_a);
        registration_b->apply(color_b, depth_b, &undistorted_b, &registered_b);

        registration_a->undistortDepth(depth_a, &undistorted_a);
        registration_b->undistortDepth(depth_b, &undistorted_b);
        
        
        pcl::PointCloud<pcl::PointXYZRGB> cloud_a;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_b;

        cloud_a.points.reserve(DEPTH_W * DEPTH_H);
        cloud_b.points.reserve(DEPTH_W * DEPTH_H);

        const int arr_len = DEPTH_W * DEPTH_H;
        const int arr_size = DEPTH_W * DEPTH_H * sizeof(float);

        sensor_msgs::PointCloud2 cloud_msg_a;
        pcl::toROSMsg(cloud_a, cloud_msg_a);
        cloud_msg_a.header.frame_id = "color_a";
        cloud_msg_a.height = 1;
        cloud_msg_a.width = DEPTH_W * DEPTH_H;
        cloud_msg_a.is_dense = false;
        cloud_msg_a.row_step = cloud_msg_a.point_step * cloud_msg_a.width;
        cloud_msg_a.data.reserve(cloud_msg_a.width * cloud_msg_a.point_step);
        cloud_msg_a.data.resize(cloud_msg_a.width * cloud_msg_a.point_step);

        sensor_msgs::PointCloud2 cloud_msg_b;
        pcl::toROSMsg(cloud_b, cloud_msg_b);
        cloud_msg_b.header.frame_id = "color_b";
        cloud_msg_b.height = 1;
        cloud_msg_b.width = DEPTH_W * DEPTH_H;
        cloud_msg_b.is_dense = false;
        cloud_msg_b.row_step = cloud_msg_b.point_step * cloud_msg_b.width;
        cloud_msg_b.data.reserve(cloud_msg_b.width * cloud_msg_b.point_step);
        cloud_msg_b.data.resize(cloud_msg_b.width * cloud_msg_b.point_step);

        kin_dev_a.getPointCloudGpu((const float*)undistorted_a.data, (const uint32_t*)registered_a.data, (uint8_t*)cloud_msg_a.data.data(), DEPTH_W, DEPTH_H);
        kin_dev_b.getPointCloudGpu((const float*)undistorted_b.data, (const uint32_t*)registered_b.data, (uint8_t*)cloud_msg_b.data.data(), DEPTH_W, DEPTH_H);

        ros::Time now = ros::Time::now();

        cloud_msg_a.header.stamp = now;
        pub_cloud_a.publish(cloud_msg_a);

        cloud_msg_b.header.stamp = now;
        pub_cloud_b.publish(cloud_msg_b);

        kin_dev_a.release_frames();
        kin_dev_b.release_frames();
    }
    kin_dev_a.stop();
    kin_dev_b.stop();
    return 0;
}
