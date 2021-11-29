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

// roslaunch aruco_detect aruco_detect.launch camera:=/color image:=/image_raw dictionary:=1 fiducial_len:=0.120 publish_images:=true
// roslaunch aruco_detect aruco_detect.launch camera:=/color_b image:=/image_raw dictionary:=1 fiducial_len:=0.120 publish_images:=true

int main(int argc, char** argv) {
    //libfreenect2::setGlobalLogger(NULL);
    std::string serial = "175175234247";
    ros::init(argc, argv, "kinect_camera");
    bool show = false;
    bool info = true;
    try {
        KinectStreamer::KinectDevice kin_dev(serial);
        if (!kin_dev.start()) {
            std::cout << "Failed to start Kinect." << std::endl;
            exit(-1);
        }
        ros::NodeHandle nh_camera_info_color;
        ros::NodeHandle nh_cloud;
        
        ros::NodeHandle nh_camera_color(nh_camera_color, "color");
        camera_info_manager::CameraInfoManager manager_color(nh_camera_color, "color");
        
        ros::Publisher pub_camera_info_color = nh_camera_info_color.advertise<sensor_msgs::CameraInfo>("color/camera_info", 1);
        if (!manager_color.loadCameraInfo("")) {
            std::cout << "Failed to get calibration from Color .yaml" << std::endl;
        }
        if (!manager_color.isCalibrated()) {
            std::cout << "Color is not calibrated." << std::endl;
        }
        ros::NodeHandle nh_color;
        image_transport::ImageTransport it_color(nh_color);
        image_transport::Publisher pub_color = it_color.advertise("color/image_raw", 1);



        ros::Publisher pub_cloud = nh_cloud.advertise<sensor_msgs::PointCloud2>("color/points", 1);


        if (show) {
            cv::namedWindow("Color", cv::WINDOW_NORMAL);
        }
        while (ros::ok()) {
            kin_dev.wait_frames();
            libfreenect2::Frame* color = kin_dev.get_frame(libfreenect2::Frame::Color);
            libfreenect2::Frame* depth = kin_dev.get_frame(libfreenect2::Frame::Depth);

            // Create Image from Color Frame
            cv::Mat img_color(cv::Size(color->width, color->height), CV_8UC4, color->data);
            // Create Image from Depth Frame
            cv::Mat img_depth(cv::Size(depth->width, depth->height), CV_32FC1, depth->data);
            // Flip the Image for Aruco Detection
            cv::flip(img_color, img_color, 1);

            // Send Image
            sensor_msgs::ImagePtr msg_color = cv_bridge::CvImage(std_msgs::Header(), "bgra8", img_color).toImageMsg();
            pub_color.publish(msg_color);
            // Send Camera Info
            sensor_msgs::CameraInfo info_camera_color = manager_color.getCameraInfo();
            info_camera_color.header.stamp = ros::Time::now();
            pub_camera_info_color.publish(info_camera_color);

            // Flip the Image Back
            cv::flip(img_color, img_color, 1);

            libfreenect2::Frame undistorted(512, 424, 4);
            libfreenect2::Frame registered(512, 424, 4);


            libfreenect2::Registration* registration = kin_dev.get_registration();
            registration->apply(color, depth, &undistorted, &registered);
            registration->undistortDepth(depth, &undistorted);

            cv::Mat img_registered(cv::Size(registered.width, registered.height), CV_8UC4, registered.data);
            cv::imshow("Registered", img_registered);
            cv::waitKey(1);

            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            for (int i = 0; i < depth->height; i++) {
                for (int j = 0; j < depth->width; j++ ) {
                    float d = ((float*)(undistorted.data))[i * depth->width + j];

                    pcl::PointXYZRGB point;
                    float rgb;
                    
                    float x_raw = 0;
                    float y_raw = 0;
                    float z_raw = 0;
                    registration->getPointXYZRGB(&undistorted, &registered, i, j, x_raw, y_raw, z_raw, rgb);
                    
                    point.x = x_raw;
                    point.y = y_raw;
                    point.z = z_raw;
                    const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
                    point.b = p[0];
                    point.g = p[1];
                    point.r = p[2];
                    cloud.points.push_back(point);
                }
            }

            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(cloud, cloud_msg);
            cloud_msg.header.frame_id = "world";
            cloud_msg.header.stamp = ros::Time::now();
            pub_cloud.publish(cloud_msg);

            if (show) {
                cv::imshow("Color", img_color);
                cv::resizeWindow("Color", cv::Size(color->width / 2, color->height / 2));
                char c = cv::waitKey(1);
                if (c == ' ') {
                    while (cv::waitKey(1) != ' ') {
                        
                    }   
                }
            }

            kin_dev.release_frames();
            
        }
    } catch (std::exception e) {
        exit(-1);
    }
}
