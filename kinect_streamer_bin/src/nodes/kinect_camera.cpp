#define ENABLE_ROS
#define ENABLE_CUDA

#include <signal.h>
#include <cstdio>
#include <cstring>
#include <argparse/argparse.hpp>
#include <kinect_streamer/kinect_streamer.hpp>

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

#ifdef ENABLE_ROS
#include <kinect_ros/kinect_ros.hpp>
#endif

#ifdef ENABLE_CUDA
#include <cuda_runtime.h>
#endif

#define myUINT8 1
#define myFLOAT32 7

#define POINT_STEP 32


#define MAX_DEPTH   (5000.0)
#define IR_DIV      (257.0)

#define DEFAULT_GPU 0


bool shutdown = false;

void sigint_handler(int s) {
  shutdown = true;
}

#ifdef ENABLE_CUDA
bool init_gpu_device(const int deviceId) {
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

    KinectCameraArgs args = argparse::parse<KinectCameraArgs>(argc, argv);

    // If not verbose, then disable the logger for libfreenect2
    if (!args.verbose) {
        libfreenect2::setGlobalLogger(NULL);
    }
    
#ifdef ENABLE_CUDA
    init_gpu_device(DEFAULT_GPU);
#endif

    ///////////////////////////////////////////////////
    //                                               //
    // Maps to store variables on a per-Kinect basis // 
    //                                               //
    ///////////////////////////////////////////////////

    // Kinect Devices (from kinect_streamer library)
    std::map<std::string, std::unique_ptr<KinectStreamer::KinectDevice>> kin_devs;

#ifdef ENABLE_ROS
    KinectROS kinect_ros(argc, argv);
#endif

    libfreenect2::Freenect2 freenect2;

    // Serial numbers serve as a key for the maps above.
    std::vector<std::string> serials;
    
    for (std::string arg : args.serials) {
        if (is_valid_serial(arg)) {
            serials.push_back(arg);
        }
    }

    // Check if there are any kinect devices, and print them out.
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
            std::cout << "Failed to start Kinect Serial no.: " << serial << "\n\r";
            exit(-1);
        }
    }

#ifdef ENABLE_ROS
    kinect_ros.init_camera_pub(serials);
#endif

    for (std::string serial : serials) {
        // For each device, initialise the intrinsic parameters from the device
        kin_devs[serial]->init_params();
        // For each device, initialise the registration object
        kin_devs[serial]->init_registration();
    }

#ifdef ENABLE_ROS
    kinect_ros.init_frame_pub(serials);
    kinect_ros.init_cloud_pub(serials);
#endif


    // Keep running while ROS is still running, and while the shutdown command is off
    while (true) {

#ifdef ENABLE_ROS
        if (!ros::ok()) {
            break;
        }
#endif

        if (shutdown) {
            break;
        }
        // Wait for the frames for all of the devices before processing.
        for (std::string serial : serials) {
            kin_devs[serial]->wait_frames();
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

#ifdef ENABLE_ROS
            kinect_ros.send_image(serial, img_color, img_ir_mono8);
#endif
            
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
            
            uint8_t cloud_data[DEPTH_W * DEPTH_H * POINT_STEP] = {0};
            
#ifdef ENABLE_CUDA
            kin_devs[serial]->getPointCloudCuda((const float*)undistorted->data, (const uint32_t*)registered->data, cloud_data, DEPTH_W, DEPTH_H);
#else
            kin_devs[serial]->getPointCloudCpu((const float*)undistorted->data, (const uint32_t*)registered->data, cloud_data, DEPTH_W, DEPTH_H);
#endif

#ifdef ENABLE_ROS
            kinect_ros.send_cloud(serial, cloud_data, sizeof(cloud_data));
#endif

            kin_devs[serial]->release_frames();

        }
    }
    for (std::string serial : serials) {
        kin_devs[serial]->stop();
    }
    return 0;
}
