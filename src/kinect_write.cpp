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
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

int flag = false;

void pre_handler(int s) {
    std::cout << "Quitting!" << std::endl;
    exit(-1);
}


void my_handler(int s) {
    std::cout << "Quitting!" << std::endl;
    flag = true;
}

int main(int argc, char** argv) {

    signal(SIGINT, pre_handler);
    std::string serial = "";
    if (argc == 2) {
        serial = argv[1];
    } else {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    if (freenect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    pipeline = new libfreenect2::CpuPacketPipeline();
    dev = freenect2.openDevice(serial, pipeline);

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);

    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    dev->startStreams(true, true);

    std::cout << "Device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "Device firmware: " << dev->getFirmwareVersion() << std::endl; 

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    signal(SIGINT, my_handler);
    namedWindow("test", WINDOW_AUTOSIZE);
    const int fps = 15;
    const int seconds = 60;
    const int max_frames = fps * seconds;
    int num_frames = 0;

    auto begin = std::chrono::high_resolution_clock::now();

    while (!flag && num_frames < max_frames) {
        if (!listener.waitForNewFrame(frames, 10 * 1000)) {
            std::cout << "Timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        int* map = (int*)calloc(sizeof(int*), 512 * 424);
        registration->apply(color, depth, &undistorted, &registered, true, NULL, map);
        std::string color_string = std::string("/media/medrobotics/Data/img/") + std::string("color/") + std::to_string(num_frames) + std::string(".bin");
        std::string depth_string = std::string("/media/medrobotics/Data/img/") + std::string("depth/") + std::to_string(num_frames) + std::string(".bin");
        std::string map_string = std::string("/media/medrobotics/Data/img/") + std::string("map/") + std::to_string(num_frames) + std::string(".bin");
        FILE* f_color = fopen(color_string.c_str(), "w+");
        if (f_color == NULL) {
            std::cout << "Color file does not exist" << std::endl;
            return -1;
        }
        FILE* f_depth = fopen(depth_string.c_str(), "w+");
        if (f_depth == NULL) {
            std::cout << "Depth file does not exist" << std::endl;
            return -1;
        }
        FILE* f_map = fopen(map_string.c_str(), "w+");
        if (f_map == NULL) {
            std::cout << "Map file does not exist" << std::endl;
            return -1;
        }
        fwrite(color->data, color->bytes_per_pixel, color->width * color->height, f_color);
        fwrite(depth->data, depth->bytes_per_pixel, depth->width * depth->height, f_depth);
        fwrite(map, 4, 512 * 424, f_map);
        free(map);
        fclose(f_color);
        fclose(f_depth);
        fclose(f_map);
        num_frames++;
        listener.release(frames);
    }
    auto end = std::chrono::high_resolution_clock::now();    
    auto duration = end - begin;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    std::cout << "Time = " << ms << " ms" << std::endl;
    std::cout << "Frames = " << num_frames << std::endl;
    std::cout << "Framerate = " << (double)num_frames / ((double)ms / 1000.0) << " FPS" << std::endl;
    dev->stop();
    dev->close();
    return 0;
}