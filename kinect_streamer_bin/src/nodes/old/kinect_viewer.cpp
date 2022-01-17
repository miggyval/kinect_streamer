#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <thread>
#include <sys/types.h>
#include <sys/stat.h>
#include <experimental/filesystem>
#include <argparse/argparse.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <kinect_streamer/kinect_streamer.hpp>

struct KinectViewerArgs : public argparse::Args {
    std::string &src_path = arg("directory path for recorded data");
};

using namespace cv;

namespace fs = std::experimental::filesystem;
int main(int argc, char** argv) {

    KinectViewerArgs args = argparse::parse<KinectViewerArgs>(argc, argv);

    int num_frames = 0;
    unsigned int time = 0;
    unsigned int last_time = 0;

    std::string root_string = args.src_path;
    std::string time_string = root_string + std::string("/time/");
    std::string color_string = root_string + std::string("/color/");
    std::string depth_string = root_string + std::string("/depth/");
    std::string ext_string = std::string(".bin");

    if (!fs::exists(fs::path(root_string))) {
        std::cout << "Cannot find 'root' folder." << std::endl;
        std::cout << "The following path does not exist: " << root_string << std::endl;
        exit(-1);
    }
    if (!fs::is_directory(root_string)) {
        std::cout << "The following path is not a folder: " << root_string << std::endl;
        exit(-1);
    }
    if (!fs::exists(fs::path(time_string))) {
        std::cout << "Cannot find 'time' folder." << std::endl;
        std::cout << "The following path does not exist: " << time_string << std::endl;
        exit(-1);
    }
    if (!fs::is_directory(time_string)) {
        std::cout << "The following path is not a folder: " << time_string << std::endl;
        exit(-1);
    }
    if (!fs::exists(fs::path(color_string))) {
        std::cout << "Cannot find 'color' folder." << std::endl;
        std::cout << "The following path does not exist: " << color_string << std::endl;
        exit(-1);
    }
    if (!fs::is_directory(color_string)) {
        std::cout << "The following path is not a folder: " << color_string << std::endl;
        exit(-1);
    }
    if (!fs::exists(fs::path(depth_string))) {
        std::cout << "Cannot find 'depth' folder." << std::endl;
        std::cout << "The following path does not exist: " << depth_string << std::endl;
        exit(-1);
    }
    if (!fs::is_directory(depth_string)) {
        std::cout << "The following path is not a folder: " << depth_string << std::endl;
        exit(-1);
    }
    if (fs::is_empty(time_string)) {
        std::cout << "Time folder is empty!" << std::endl;
        exit(-1);
    }
    if (fs::is_empty(color_string)) {
        std::cout << "Color folder is empty!" << std::endl;
        exit(-1);
    }
    if (fs::is_empty(depth_string)) {
        std::cout << "Depth folder is empty!" << std::endl;
        exit(-1);
    }

    namedWindow("color", WINDOW_NORMAL);
    namedWindow("depth", WINDOW_NORMAL);
    resizeWindow("color", Size(1920 / 2, 1080 / 2));

    while (true) {
        auto current_start = std::chrono::high_resolution_clock::now();
        std::string time_filename = time_string + std::to_string(num_frames) + ext_string;
        std::string color_filename = color_string + std::to_string(num_frames) + ext_string;
        std::string depth_filename = depth_string + std::to_string(num_frames) + ext_string;
        FILE* f_time = fopen(time_filename.c_str(), "r");
        FILE* f_color = fopen(color_filename.c_str(), "r");
        FILE* f_depth = fopen(depth_filename.c_str(), "r");
        if (!f_time || !f_color || !f_depth) {
            std::cout << "Finished!" << std::endl;
            std::cout << "Exiting viewer." << std::endl;
            break;
        }
        Mat img_color(Size(COLOR_W, COLOR_H), CV_8UC4);
        Mat img_depth(Size(DEPTH_W, DEPTH_H), CV_32FC1);
        fread(img_color.data, img_color.elemSize(), img_color.rows * img_color.cols, f_color);
        fread(img_depth.data, img_depth.elemSize(), img_depth.rows * img_depth.cols, f_depth);
        last_time = time;
        fread(&time, sizeof(unsigned int), 1, f_time);
        fclose(f_color);
        fclose(f_depth);
        fclose(f_time);
        imshow("color", img_color);
        imshow("depth", img_depth / DEPTH_MAX);
        waitKey(1);
        auto current_end = std::chrono::high_resolution_clock::now();
        auto current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_end - current_start);
        while (current_duration.count() < (int)(time - last_time)) {
            current_end = std::chrono::high_resolution_clock::now();
            current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_end - current_start);
        }
        num_frames++;
    }
    cv::destroyAllWindows();
    return 0;
}