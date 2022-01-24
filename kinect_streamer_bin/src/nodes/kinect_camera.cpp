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

#include <cuda_runtime.h>


#define myUINT8 1
#define myFLOAT32 7


#define MAX_DEPTH   (5000.0)
#define IR_DIV      (257.0)


bool shutdown = false;

void sigint_handler(int s) {
  shutdown = true;
}


#define USE_CUDA

#ifdef USE_CUDA
bool initDevice(const int deviceId) {
    int deviceCount = 0;
    cudaGetDeviceCount(&deviceCount);
    int devId = -1;
    for (int i = 0; i < deviceCount; i++) {
        if (deviceId != -1 && i != deviceId) {
            continue;
        }

        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        std::cout << "device " << i << ": " << prop.name << " @ " << (prop.clockRate / 1000) << "MHz Memory " << (prop.totalGlobalMem >> 20) << "MB" << "\n\r";

        if (prop.computeMode == cudaComputeModeProhibited) {
            std::cout << " Compute Mode Prohibited" << "\n\r";
            continue;
        }

        if (prop.major < 1) {
            std::cout << " does not support CUDA" << "\n\r";
            continue;
        }

        devId = i;
        break;
    }

    if (devId == -1) {
        std::cout << "No CUDA device found" << "\n\r";
        return false;
    }

    cudaSetDevice(devId);
    std::cout << "selected device " << devId << "\n\r";

    return true;
}
#endif

/**
 * @brief Arguments for Kinect Camera Program
 * See https://github.com/morrisfranken/argparse for usage
 */
struct KinectCameraArgs : public argparse::Args {
    std::vector<std::string> &serials = kwarg("s,serials", "Serial Numbers").multi_argument();
    bool &verbose = kwarg("v,verbose", "A flag to toggle verbose", "true").set_default(false);
};


/**
 * @brief Checks if string is a valid 12-digit serial number
 * 
 * @param str possible serial number to validate
 * @return true The string is a valid serial number.
 * @return false The string is a valid serial number.
 */
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


/**
 * @brief Main logic for Kinect Camera program
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "kinect_camera");

    KinectCameraArgs args = argparse::parse<KinectCameraArgs>(argc, argv);

    if (!args.verbose) {
        libfreenect2::setGlobalLogger(NULL);
    }
    
#ifdef USE_CUDA
    initDevice(0);
#endif

    ///////////////////////////////////////////////////
    //                                               //
    // Maps to store variables on a per-Kinect basis // 
    //                                               //
    ///////////////////////////////////////////////////

    // Kinect Devices (from kinect_streamer library)
    std::map<std::string, std::unique_ptr<KinectStreamer::KinectDevice>> kin_devs;

    // Node Handles
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


    // Serial numbers serve as a key for the maps above.
    std::vector<std::string> serials;
    
    for (std::string arg : args.serials) {
        if (is_valid_serial(arg)) {
            serials.push_back(arg);
        }
    }

    // Check if there are any kinect devices, and print them out.
    libfreenect2::Freenect2 freenect2;
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

    // Check if there are more serial numbers than then available devices.
    int n = serials.size();
    if (n > num_devices) {
        std::cout << "Too many serial numbers in input." << "\n\r";
        exit(-1);
    }
    
    // For the serial numbers supplied, try to start kinect devices.
    for (std::string serial : serials) {
        kin_devs[serial] = std::make_unique<KinectStreamer::KinectDevice>(serial);
        if (!kin_devs[serial]->start()) {
            std::cout << "Failed to start Kinect Serial no.: " << serial << std::endl;
            exit(-1);
        }
    }

    for (std::string serial : serials) {

        // For each device, create node handles and publishers.
        nh_camera_info_colors[serial]   = std::make_unique<ros::NodeHandle>();
        nh_camera_info_irs[serial]      = std::make_unique<ros::NodeHandle>();

        nh_camera_colors[serial]        = std::make_unique<ros::NodeHandle>(std::string("color_") + serial);
        nh_camera_irs[serial]           = std::make_unique<ros::NodeHandle>(std::string("ir_") + serial);

        pub_camera_info_colors[serial]  = nh_camera_info_colors[serial]->advertise<sensor_msgs::CameraInfo>(std::string("color_") + serial + std::string("/camera_info"), 1);
        pub_camera_info_irs[serial]     = nh_camera_info_irs[serial]->advertise<sensor_msgs::CameraInfo>(std::string("ir_") + serial + std::string("/camera_info"), 1);

        // For each device, create a camera info manager using the default directory for calibration data.
        manager_colors[serial]          = std::make_unique<camera_info_manager::CameraInfoManager>(*nh_camera_colors[serial], std::string("color_") + serial);
        manager_irs[serial]             = std::make_unique<camera_info_manager::CameraInfoManager>(*nh_camera_irs[serial], std::string("ir_") + serial);

        if (!manager_colors[serial]->loadCameraInfo("")) {
            std::cout << "Failed to get calibration for color camera: " << serial << std::endl;
        }

        if (!manager_irs[serial]->loadCameraInfo("")) {
            std::cout << "Failed to get calibration from ir camera: " << serial << std::endl;
        }

        info_colors[serial] = manager_colors[serial]->getCameraInfo();
        info_irs[serial]    = manager_irs[serial]->getCameraInfo();
    }
    

    for (std::string serial : serials) {
        // For each device, initialise the intrinsic parameters from the device
        kin_devs[serial]->init_params();
        // For each device, initialise the registration object
        kin_devs[serial]->init_registration();
    }
    
    for (std::string serial : serials) {
        
        // Node Handles
        nh_clouds[serial]   = std::make_unique<ros::NodeHandle>();  // Point Cloud Node Handle

        it_colors[serial]   = std::make_unique<image_transport::ImageTransport>(ros::NodeHandle()); // Image Transport Node Handle for Color Frame
        it_irs[serial]      = std::make_unique<image_transport::ImageTransport>(ros::NodeHandle()); // Image Transport Node Handle for IR Frame

        pub_clouds[serial]  = nh_clouds[serial]->advertise<sensor_msgs::PointCloud2>(std::string("/points_") + serial, 1);  // Publisher for Point Cloud
        pub_colors[serial]  = it_colors[serial]->advertise(std::string("color_") + serial + "/image_raw", 1);               // Publisher for Color Frame
        pub_irs[serial]     = it_irs[serial]->advertise(std::string("ir_") + serial + "/image_raw", 1);                     // Publisher for IR Frame
    }

    ros::Time now = ros::Time::now();

    // Keep running while ROS is still running, and while the shutdown command is off
    while (ros::ok() && !shutdown) {
        
        // Wait for the frames for all of the devices before processing.
        for (std::string serial : serials) {
            kin_devs[serial]->wait_frames();
        }
        // Broadcast the transforms based on extrinsic parameters
        // TODO: Defaulting to z-axis (of kinect) facing forward
        for (std::string serial : serials) {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setIdentity();
            transform.setRotation(tf::createQuaternionFromRPY(-3.141592 / 2, 0, 0));
            now = ros::Time::now();
            br.sendTransform(tf::StampedTransform(transform, now, "world", std::string("frame_") + serial));
        }

        // Process the frame data (color, depth, and ir)
        for (std::string serial : serials) {
            
            // Get the frame data from the device
            libfreenect2::Frame* color  = kin_devs[serial]->get_frame(libfreenect2::Frame::Color);
            libfreenect2::Frame* depth  = kin_devs[serial]->get_frame(libfreenect2::Frame::Depth);
            libfreenect2::Frame* ir     = kin_devs[serial]->get_frame(libfreenect2::Frame::Ir);
            
            // Convert the data into OpenCV format
            cv::Mat img_color(cv::Size(color->width, color->height), CV_8UC4, color->data);
            cv::Mat img_depth(cv::Size(depth->width, depth->height), CV_32FC1, depth->data);
            cv::Mat img_ir(cv::Size(ir->width, ir->height), CV_32FC1, ir->data);



            // Normalize the IR data for viewing [0, 65535.0] -> [0, 255.0]
            cv::Mat img_ir_div;
            cv::divide(IR_DIV, img_ir, img_ir_div);
            cv::Mat img_ir_mono8;
            img_ir_div.convertTo(img_ir_mono8, CV_8UC1);

            // Flip cameras (automatically flipped by Kinect)
            cv::flip(img_color, img_color, 1);
            cv::flip(img_ir_mono8, img_ir_mono8, 1);

            // Create ROS sensor messages from OpenCV Images
            sensor_msgs::ImagePtr msg_color = cv_bridge::CvImage(std_msgs::Header(), "bgra8", img_color).toImageMsg();
            sensor_msgs::ImagePtr msg_ir    = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_ir_mono8).toImageMsg();

            // Publish the sensor messages
            pub_colors[serial].publish(msg_color);
            pub_irs[serial].publish(msg_ir);

            // Create and publish the camera info
            sensor_msgs::CameraInfo info_camera_color = manager_colors[serial]->getCameraInfo();
            sensor_msgs::CameraInfo info_camera_ir = manager_irs[serial]->getCameraInfo();
            now = ros::Time::now();
            info_camera_color.header.stamp = now;
            info_camera_ir.header.stamp = now;
            pub_camera_info_colors[serial].publish(info_camera_color);
            pub_camera_info_irs[serial].publish(info_camera_ir);
            
            // Flip color camera back (IR camera is not used again)
            cv::flip(img_color, img_color, 1);

            // Create frames for undistorted (depth) and registered images (color)
            std::unique_ptr<libfreenect2::Frame> undistorted = std::make_unique<libfreenect2::Frame>(512, 424, 4);
            std::unique_ptr<libfreenect2::Frame> registered = std::make_unique<libfreenect2::Frame>(512, 424, 4);
            cv::Mat img_undistorted(cv::Size(undistorted->width, undistorted->height), CV_32FC1, undistorted->data);
            cv::Mat img_registered(cv::Size(registered->width, registered->height), CV_8UC4, registered->data);
    
            // Use image registration to get the undistorted and registered images.
            libfreenect2::Registration* registration = kin_devs[serial]->get_registration();
            registration->apply(color, depth, undistorted.get(), registered.get());
            registration->undistortDepth(depth, undistorted.get());
            
            // Create point cloud data structure
            std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
            cloud->points.reserve(DEPTH_W * DEPTH_H);
            const int arr_len = DEPTH_W * DEPTH_H;
            const int arr_size = DEPTH_W * DEPTH_H * sizeof(float);
            
            std::unique_ptr<sensor_msgs::PointCloud2> cloud_msg = std::make_unique<sensor_msgs::PointCloud2>();
            pcl::toROSMsg(*cloud, *cloud_msg);

            // Set the parameters of the point cloud
            cloud_msg->header.frame_id = std::string("frame_") + serial;
            cloud_msg->height = 1;
            cloud_msg->width = DEPTH_W * DEPTH_H;
            cloud_msg->is_dense = false;
            cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
            cloud_msg->data.reserve(cloud_msg->width * cloud_msg->point_step);
            cloud_msg->data.resize(cloud_msg->width * cloud_msg->point_step);
#ifdef USE_CUDA
            kin_devs[serial]->getPointCloudCuda((const float*)undistorted->data, (const uint32_t*)registered->data, (uint8_t*)cloud_msg->data.data(), DEPTH_W, DEPTH_H);
#else
            kin_devs[serial]->getPointCloudCpu((const float*)undistorted->data, (const uint32_t*)registered->data, (uint8_t*)cloud_msg->data.data(), DEPTH_W, DEPTH_H);
#endif
            now = ros::Time::now();
            cloud_msg->header.stamp = now;
            pub_clouds[serial].publish(*cloud_msg);
            
            kin_devs[serial]->release_frames();

        }
    }
    for (std::string serial : serials) {
        kin_devs[serial]->stop();
    }
    return 0;
}
