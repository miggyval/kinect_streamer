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
#include <sys/types.h>
#include <sys/stat.h>
#include <experimental/filesystem>
#include <argparse/argparse.hpp>

struct KinectRecorderArgs : public argparse::Args {
    std::string &src_path = arg("save directory path");
    bool &webcam = flag("w,webcam", "use webcam instead of kinect").set_default(false);
};

int flag = false;

void pre_handler(int s) {
    std::cout << "Quitting!" << std::endl;
    exit(-1);
}


void my_handler(int s) {
    std::cout << "Quitting!" << std::endl;
    flag = true;
}


namespace fs = std::experimental::filesystem;

void iter_delete(fs::path path) {
    std::cout << "Delete?" << std::endl;
    std::cin.ignore();
}

int main(int argc, char** argv) {

    KinectRecorderArgs args = argparse::parse<KinectRecorderArgs>(argc, argv);

    signal(SIGINT, pre_handler);

    std::string root_string = std::string(argv[1]);
    std::string time_string = root_string + std::string("/time/");
    std::string color_string = root_string + std::string("/color/");
    std::string depth_string = root_string + std::string("/depth/");
    std::string map_string = root_string + std::string("/map/");
    std::string ext_string = std::string(".bin");


    if (fs::exists(fs::path(root_string.c_str()))) {
        std::cout << "Folder already exists." << std::endl;
        std::cout << "Would you like to overwrite the data? [Y/n]" << std::endl;
        std::string yn;
        while (true) {
            std::cin >> yn;
            if (yn == std::string("Y") || yn == std::string("y")) {
                break;
            } else if (yn == std::string("N") || yn == std::string("n")) {
                exit(1);
            }
            std::cout << "Please try again." << std::endl;
        }
        std::cout << "Erasing ALL contents of " << root_string << std::endl;
        iter_delete("");
    } else {
        std::cout << "Creating folder: " << root_string << std::endl;
        bool root_check = fs::create_directories(fs::path(root_string));
    }

    int color_check = mkdir(color_string.c_str(), 0777);
    int depth_check = mkdir(depth_string.c_str(), 0777);
    int map_check = mkdir(map_string.c_str(), 0777);
    int time_check = mkdir(time_string.c_str(), 0777);

    std::string serial = "";

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    serial = freenect2.getDefaultDeviceSerialNumber();

    if (freenect2.enumerateDevices() == 0) {
        std::cout << "No device connected!" << std::endl;
        return -1;
    }
    pipeline = new libfreenect2::OpenGLPacketPipeline();
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
    const int fps = 15;
    const int seconds = 60 * 30;
    const int max_frames = fps * seconds;
    int num_frames = 0;

    auto begin = std::chrono::high_resolution_clock::now();

    while (!flag && num_frames < max_frames) {
        if (!listener.waitForNewFrame(frames, 10 * 1000)) {
            std::cout << "Timeout!" << std::endl;
            return -1;
        }

        std::string color_filename = color_string + std::to_string(num_frames) + ext_string;
        std::string depth_filename = depth_string + std::to_string(num_frames) + ext_string;
        std::string map_filename = map_string + std::to_string(num_frames) + ext_string;
        std::string time_filename = time_string + std::to_string(num_frames) + ext_string;

	    auto now = std::chrono::high_resolution_clock::now() - begin;
	    auto mill = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
	    libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        int* map = (int*)calloc(sizeof(int*), 512 * 424);
        registration->apply(color, depth, &undistorted, &registered, true, NULL, map);

        FILE* f_color = fopen(color_filename.c_str(), "w+");
        FILE* f_depth = fopen(depth_filename.c_str(), "w+");
        FILE* f_map = fopen(map_filename.c_str(), "w+");
        FILE* f_time = fopen(time_filename.c_str(), "w+");

        if (!f_color || !f_depth || !f_map || !f_time) {
            std::cout << "Invalid filename" << std::endl;
            break;
        }

        fwrite(color->data, color->bytes_per_pixel, color->width * color->height, f_color);
        fwrite(depth->data, depth->bytes_per_pixel, depth->width * depth->height, f_depth);
        fwrite(&mill, sizeof(mill), 1, f_time);
	    fwrite(map, 4, 512 * 424, f_map);
	
        free(map);
        fclose(f_color);
        fclose(f_depth);
	    fclose(f_time);
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
