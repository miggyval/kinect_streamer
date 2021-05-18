#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

int flag = false;

void my_handler(int s) {
    std::cout << "Exiting!" << std::endl;
    flag = true;
}

int main(int argc, char** argv) {   

    const int fps = 15;
    const int seconds = 60;
    const int max_frames = fps * seconds;
    int num_frames = 0;
    namedWindow("color", WINDOW_AUTOSIZE);
    namedWindow("depth", WINDOW_AUTOSIZE);
    while (num_frames < max_frames) {
        std::string color_string = std::string("img/color/") + std::to_string(num_frames) + std::string(".bin");
        std::string depth_string = std::string("img/depth/") + std::to_string(num_frames) + std::string(".bin");
        FILE* f_color = fopen(color_string.c_str(), "r");
        FILE* f_depth = fopen(depth_string.c_str(), "r");
        Mat img_color(Size(1920, 1080), CV_8UC4);
        Mat img_depth(Size(512, 424), CV_32FC1);
        fread(img_color.data, img_color.elemSize(), img_color.rows * img_color.cols, f_color);
        fread(img_depth.data, img_depth.elemSize(), img_depth.rows * img_depth.cols, f_depth);
        fclose(f_color);
        fclose(f_depth);
        imshow("color", img_color);
        imshow("depth", img_depth / 2000.0);
        waitKey(1000.0 / 7.5);
        num_frames++;
    }
    return 0;
}