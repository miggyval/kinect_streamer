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
#include <argparse/argparse.hpp>


#include <tf/transform_broadcaster.h>

#define myUINT8 1
#define myFLOAT32 7


#define MAX_DEPTH 5000.0

bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s) {
  protonect_shutdown = true;
}

// roslaunch aruco_detect aruco_detect.launch camera:=/color image:=/image_raw dictionary:=1 fiducial_len:=0.120 publish_images:=true
// roslaunch aruco_detect aruco_detect.launch camera:=/color_b image:=/image_raw dictionary:=1 fiducial_len:=0.120 publish_images:=true


struct KinectCameraArgs : public argparse::Args {
    std::vector<std::string> &serials = kwarg("s,serials", "Serial Numbers").multi_argument();
    bool &verbose = kwarg("v,verbose", "A flag to toggle verbose", "false").set_default(false);
};



bool is_valid_serial(std::string str) {
    std::locale loc;
    const char* buffer = str.c_str();
    int count = 0;
    char c = buffer[count];
    while (c != 0) {
        if (c < '0' && c > '9') {
            return false;
        }
        count++;
        c = buffer[count];
    }
    if (count != 12) {
        return false;
    }
    return true;
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "kinect_camera");
    libfreenect2::setGlobalLogger(NULL);

    ////


    KinectCameraArgs args = argparse::parse<KinectCameraArgs>(argc, argv);


    if (!args.verbose) {
        libfreenect2::setGlobalLogger(NULL);
    }
    libfreenect2::Freenect2 freenect2;
    std::map<std::string, KinectStreamer::KinectDevice*> kin_devs;
    std::vector<std::string> serial_args = args.serials;
    std::vector<std::string> serials;
    for (std::string arg : serial_args) {
        if (is_valid_serial(arg)) {
            serials.push_back(arg);
        }
    }

    int num_devices = freenect2.enumerateDevices();
    if (num_devices == 0) {
        std::cout << "No devices detected!" << "\n\r";
        exit(-1);
    } else {
        std::cout << "Connected devices:" << "\n\r";
        for (int idx = 0; idx < num_devices; idx++) {
            std::cout << "- " << freenect2.getDeviceSerialNumber(idx) << "\n\r";
        }
    }


    int n = serials.size();
    if (n > num_devices) {
        std::cout << "Too many serial numbers in input." << "\n\r";
        exit(-1);
    }
    bool show = false;
    bool info = true;


    for (std::string serial : serials) {
        KinectStreamer::KinectDevice* kin_dev = new KinectStreamer::KinectDevice(serial);
        if (!kin_dev->start()) {
            std::cout << "Failed to start Kinect Serial no.: " << serial << std::endl;
            exit(-1);
        }
        kin_devs[serial] = kin_dev;
    }
    std::map<std::string, ros::NodeHandle*> nh_camera_info_colors;
    std::map<std::string, ros::NodeHandle*> nh_camera_info_irs;
    std::map<std::string, ros::NodeHandle*> nh_camera_colors;
    std::map<std::string, ros::NodeHandle*> nh_camera_irs;
    std::map<std::string, camera_info_manager::CameraInfoManager*> manager_colors;
    std::map<std::string, camera_info_manager::CameraInfoManager*> manager_irs;
    std::map<std::string, ros::Publisher> pub_camera_info_colors;
    std::map<std::string, ros::Publisher> pub_camera_info_irs;
    std::map<std::string, sensor_msgs::CameraInfo> info_colors;
    std::map<std::string, sensor_msgs::CameraInfo> info_irs;
    for (std::string serial : serials) {
        nh_camera_info_colors[serial] = new ros::NodeHandle();
        nh_camera_info_irs[serial] = new ros::NodeHandle();
        nh_camera_colors[serial] = new ros::NodeHandle(std::string("color_") + serial);
        nh_camera_irs[serial] = new ros::NodeHandle(std::string("ir_") + serial);
        manager_colors[serial] = new camera_info_manager::CameraInfoManager(*nh_camera_colors[serial], std::string("color_") + serial);
        manager_irs[serial] = new camera_info_manager::CameraInfoManager(*nh_camera_irs[serial], std::string("ir_") + serial);
        pub_camera_info_colors[serial] = nh_camera_info_colors[serial]->advertise<sensor_msgs::CameraInfo>(std::string("color_") + serial + std::string("/camera_info"), 1);
        pub_camera_info_irs[serial] = nh_camera_info_irs[serial]->advertise<sensor_msgs::CameraInfo>(std::string("ir_") + serial + std::string("/camera_info"), 1);

        if (!manager_colors[serial]->loadCameraInfo("")) {
            std::cout << "Failed to get calibration for color camera: " << serial << std::endl;
        }
        if (!manager_irs[serial]->loadCameraInfo("")) {
            std::cout << "Failed to get calibration from ir camera: " << serial << std::endl;
        }
        info_colors[serial] = manager_colors[serial]->getCameraInfo();
        info_irs[serial] = manager_irs[serial]->getCameraInfo();
    }
    
    for (std::string serial : serials) {
        kin_devs[serial]->init_params();
    }

/*
    std::map<std::string, float> cx_colors, cy_colors, fx_colors, fy_colors;
    for (std::string serial : serials) {
        kin_devs[serial]->get_color_params(cx_colors[serial], cy_colors[serial], fx_colors[serial], fy_colors[serial]);
        info_colors[serial].K[2] = cx_colors[serial];
        info_colors[serial].K[5] = cy_colors[serial];
        info_colors[serial].K[0] = fx_colors[serial];
        info_colors[serial].K[4] = fy_colors[serial];
        info_colors[serial].K[2] = cx_colors[serial];
        info_colors[serial].K[5] = cy_colors[serial];
        info_colors[serial].K[0] = fx_colors[serial];
        info_colors[serial].K[4] = fy_colors[serial];
        manager_colors[serial]->setCameraInfo(info_colors[serial]);
    }

    std::map<std::string, float> cx_irs, cy_irs, fx_irs, fy_irs, k1_irs, k2_irs, k3_irs, p1_irs, p2_irs;
    for (std::string serial : serials) {
        kin_devs[serial]->get_ir_params(cx_irs[serial], cy_irs[serial], fx_irs[serial], fy_irs[serial], k1_irs[serial], k2_irs[serial], k3_irs[serial], p1_irs[serial], p2_irs[serial]);
        info_irs[serial].K[2] = cx_irs[serial];
        info_irs[serial].K[5] = cy_irs[serial];
        info_irs[serial].K[0] = fx_irs[serial];
        info_irs[serial].K[4] = fy_irs[serial];
        info_irs[serial].D[0] = p1_irs[serial];
        info_irs[serial].D[1] = p2_irs[serial];
        info_irs[serial].D[2] = k1_irs[serial];
        info_irs[serial].D[3] = k2_irs[serial];
        info_irs[serial].D[4] = k3_irs[serial];
        manager_irs[serial]->setCameraInfo(info_irs[serial]);
    }
    */
    for (std::string serial : serials) {
        kin_devs[serial]->init_registration();
    }
    std::map<std::string, ros::NodeHandle*> nh_clouds;
    std::map<std::string, ros::NodeHandle*> nh_colors;
    std::map<std::string, ros::NodeHandle*> nh_irs;
    std::map<std::string, image_transport::ImageTransport*> it_colors;
    std::map<std::string, image_transport::ImageTransport*> it_irs;
    std::map<std::string, ros::Publisher> pub_clouds;
    std::map<std::string, image_transport::Publisher> pub_colors;
    std::map<std::string, image_transport::Publisher> pub_irs;


    for (std::string serial : serials) {
        nh_clouds[serial] = new ros::NodeHandle();
        nh_colors[serial] = new ros::NodeHandle();
        nh_irs[serial] = new ros::NodeHandle();
        it_colors[serial] = new image_transport::ImageTransport(*nh_colors[serial]);
        it_irs[serial] = new image_transport::ImageTransport(*nh_irs[serial]);
        pub_clouds[serial] = nh_clouds[serial]->advertise<sensor_msgs::PointCloud2>(std::string("/points_") + serial, 1);
        pub_colors[serial] = it_colors[serial]->advertise(std::string("color_") + serial + "/image_raw", 1);
        pub_irs[serial] = it_irs[serial]->advertise(std::string("ir_") + serial + "/image_raw", 1);
    }

    while (ros::ok() && !protonect_shutdown) {
        
        for (std::string serial : serials) {
            kin_devs[serial]->wait_frames();
        }


        for (std::string serial : serials) {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setIdentity();
            transform.setRotation(tf::createQuaternionFromRPY(-3.141592 / 2, 0, 0));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", std::string("frame_") + serial));
        }
        for (std::string serial : serials) {
            libfreenect2::Frame* color = kin_devs[serial]->get_frame(libfreenect2::Frame::Color);
            libfreenect2::Frame* depth = kin_devs[serial]->get_frame(libfreenect2::Frame::Depth);
            libfreenect2::Frame* ir = kin_devs[serial]->get_frame(libfreenect2::Frame::Ir);
            cv::Mat img_color(cv::Size(color->width, color->height), CV_8UC4, color->data);
            cv::Mat img_depth(cv::Size(depth->width, depth->height), CV_32FC1, depth->data);
            cv::Mat img_ir(cv::Size(ir->width, ir->height), CV_32FC1, ir->data);
            img_ir /= 128.0;
            cv::Mat img_ir_mono8;
            img_ir.convertTo(img_ir_mono8, CV_8UC1);
            cv::flip(img_color, img_color, 1);
            sensor_msgs::ImagePtr msg_color = cv_bridge::CvImage(std_msgs::Header(), "bgra8", img_color).toImageMsg();
            sensor_msgs::ImagePtr msg_ir = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_ir_mono8).toImageMsg();
            pub_colors[serial].publish(msg_color);
            pub_irs[serial].publish(msg_ir);
            sensor_msgs::CameraInfo info_camera_color = manager_colors[serial]->getCameraInfo();
            info_camera_color.header.stamp = ros::Time::now();
            pub_camera_info_colors[serial].publish(info_camera_color);
            sensor_msgs::CameraInfo info_camera_ir = manager_irs[serial]->getCameraInfo();
            info_camera_ir.header.stamp = ros::Time::now();
            pub_camera_info_irs[serial].publish(info_camera_ir);
            
            cv::flip(img_color, img_color, 1);
            std::unique_ptr<libfreenect2::Frame> undistorted = std::make_unique<libfreenect2::Frame>(512, 424, 4);
            std::unique_ptr<libfreenect2::Frame> registered = std::make_unique<libfreenect2::Frame>(512, 424, 4);
            cv::Mat img_undistorted(cv::Size(undistorted->width, undistorted->height), CV_32FC1, undistorted->data);
            cv::Mat img_registered(cv::Size(registered->width, registered->height), CV_8UC4, registered->data);
            libfreenect2::Registration* registration = kin_devs[serial]->get_registration();
            cv::imshow(std::string("undistorted_") + serial, img_undistorted / 1024);
            cv::imshow(std::string("registered_") + serial, img_registered);
            cv::waitKey(1);

            
            registration->apply(color, depth, undistorted.get(), registered.get());
            
            registration->undistortDepth(depth, undistorted.get());
            
            std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
            cloud->points.reserve(DEPTH_W * DEPTH_H);
            const int arr_len = DEPTH_W * DEPTH_H;
            const int arr_size = DEPTH_W * DEPTH_H * sizeof(float);
            std::unique_ptr<sensor_msgs::PointCloud2> cloud_msg = std::make_unique<sensor_msgs::PointCloud2>();
            pcl::toROSMsg(*cloud, *cloud_msg);
            cloud_msg->header.frame_id = std::string("frame_") + serial;
            cloud_msg->height = 1;
            cloud_msg->width = DEPTH_W * DEPTH_H;
            cloud_msg->is_dense = false;
            cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
            cloud_msg->data.reserve(cloud_msg->width * cloud_msg->point_step);
            cloud_msg->data.resize(cloud_msg->width * cloud_msg->point_step);
            kin_devs[serial]->getPointCloudGpu((const float*)undistorted->data, (const uint32_t*)registered->data, (uint8_t*)cloud_msg->data.data(), DEPTH_W, DEPTH_H);
            ros::Time now = ros::Time::now();
            cloud_msg->header.stamp = now;
            pub_clouds[serial].publish(*cloud_msg);
            
            kin_devs[serial]->release_frames();

        }
    }
    for (std::string serial : serials) {
        delete nh_camera_info_colors[serial];
        delete nh_camera_info_irs[serial];
        delete nh_camera_colors[serial];
        delete nh_camera_irs[serial];
        delete manager_colors[serial];
        delete manager_irs[serial];
        delete nh_clouds[serial];
        delete nh_colors[serial];
        delete nh_irs[serial];
        delete it_colors[serial];
        delete it_irs[serial];
        kin_devs[serial]->stop();
        delete kin_devs[serial];
    }
    return 0;
}
