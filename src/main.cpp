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

    std::string serial = "";
    if (argc == 2) {
        serial = argv[1];
    }
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    if (freenect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    if (serial == "") {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    pipeline = new libfreenect2::CpuPacketPipeline();
    dev = freenect2.openDevice(serial, pipeline);

    //libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color);
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
    const int max_frames = 100;
    int num_frames = 0;
    while (!flag && num_frames < max_frames) {
        if (!listener.waitForNewFrame(frames, 10 * 1000)) {
            std::cout << "Timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
        //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        //libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        //registration->apply(color, depth, &undistorted, &registered);
        
        Mat img_color(Size(color->width, color->height), CV_8UC4, color->data);
        Mat img_color_720p;
        resize(img_color, img_color_720p, Size(1280, 720));
        //Mat img_ir(Size(ir->width, ir->height), CV_32FC1, ir->data);
        //Mat img_depth(Size(depth->width, depth->height), CV_32FC1, depth->data);
        //cvtColor(img_color, img_color, COLOR_BGRA2BGR);
        //Mat img_rand(Size(1920, 1080), CV_8UC3);
        //randu(img_rand, Scalar(0, 0, 0), Scalar(255, 255, 255));
        //imshow("test", img_color);
        imwrite(std::string("img/img_") + std::to_string(num_frames) + std::string(".png"), img_color_720p, {IMWRITE_PNG_COMPRESSION, 1, IMWRITE_PNG_STRATEGY_HUFFMAN_ONLY, IMWRITE_PNG_STRATEGY_HUFFMAN_ONLY});
        num_frames++;
        std::cout << num_frames << std::endl;
        //waitKey(1);
        listener.release(frames);
    }
    dev->stop();
    dev->close();
    return 0;
}