#include <kinect_ros/kinect_ros.hpp>

KinectROS::KinectROS(int argc, char** argv) {
    ros::init(argc, argv, "kinect_camera");
}

void KinectROS::init_camera_pub(std::vector<std::string> serials) {
    // For each device, create node handles and publishers.
    for (std::string serial : serials) {

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
}


void KinectROS::init_frame_pub(std::vector<std::string> serials) {
    for (std::string serial : serials) {

        it_colors[serial]   = std::make_unique<image_transport::ImageTransport>(ros::NodeHandle()); // Image Transport Node Handle for Color Frame
        it_irs[serial]      = std::make_unique<image_transport::ImageTransport>(ros::NodeHandle()); // Image Transport Node Handle for IR Frame

        pub_colors[serial]  = it_colors[serial]->advertise(std::string("color_") + serial + "/image_raw", 1);               // Publisher for Color Frame
        pub_irs[serial]     = it_irs[serial]->advertise(std::string("ir_") + serial + "/image_raw", 1);                     // Publisher for IR Frame
    }
}


void KinectROS::init_cloud_pub(std::vector<std::string> serials) {
    for (std::string serial : serials) {
        nh_clouds[serial]   = std::make_unique<ros::NodeHandle>();  // Point Cloud Node Handle
        pub_clouds[serial]  = nh_clouds[serial]->advertise<sensor_msgs::PointCloud2>(std::string("/points_") + serial, 1);  // Publisher for Point Cloud
    }
}


void KinectROS::send_image(std::string serial, cv::Mat img_color, cv::Mat img_ir_mono8) {
    // Create ROS sensor messages from OpenCV Images
    sensor_msgs::ImagePtr msg_color = cv_bridge::CvImage(std_msgs::Header(), "bgra8", img_color).toImageMsg();
    sensor_msgs::ImagePtr msg_ir    = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_ir_mono8).toImageMsg();

    // Publish the sensor messages
    pub_colors[serial].publish(msg_color);
    pub_irs[serial].publish(msg_ir);

    // Create and publish the camera info
    sensor_msgs::CameraInfo info_camera_color = manager_colors[serial]->getCameraInfo();
    sensor_msgs::CameraInfo info_camera_ir = manager_irs[serial]->getCameraInfo();
    ros::Time now = ros::Time::now();
    info_camera_color.header.stamp = now;
    info_camera_ir.header.stamp = now;
    pub_camera_info_colors[serial].publish(info_camera_color);
    pub_camera_info_irs[serial].publish(info_camera_ir);
}


void KinectROS::send_cloud(std::string serial, uint8_t* cloud_data, int size) {
    ros::Time now = ros::Time::now();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setIdentity();
    transform.setRotation(tf::createQuaternionFromRPY(-3.141592 / 2, 0, 0));
    br.sendTransform(tf::StampedTransform(transform, now, "world", std::string("frame_") + serial));

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.points.reserve(DEPTH_W * DEPTH_H);
    const int arr_len = DEPTH_W * DEPTH_H;
    const int arr_size = DEPTH_W * DEPTH_H * sizeof(float);
    std::unique_ptr<sensor_msgs::PointCloud2> cloud_msg = std::make_unique<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(cloud, *cloud_msg);
    cloud_msg->header.frame_id = std::string("frame_") + serial;
    cloud_msg->height = 1;
    cloud_msg->width = DEPTH_W * DEPTH_H;
    cloud_msg->is_dense = false;
    cloud_msg->row_step = DEPTH_W * DEPTH_H * POINT_STEP;
    cloud_msg->data.reserve(DEPTH_W * DEPTH_H * POINT_STEP);
    cloud_msg->data.resize(DEPTH_W * DEPTH_H * POINT_STEP);
    memcpy(cloud_msg->data.data(), cloud_data, size);
    cloud_msg->header.stamp = now;
    pub_clouds[serial].publish(*cloud_msg);
}