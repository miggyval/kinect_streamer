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

struct KinectRecorderArgs : public argparse::Args {
    std::string &src_path = arg("directory path for recorded data");
    int &fps = kwarg("f,framerate", "framerate (fps)").set_default(15);
    bool &webcam = flag("w,webcam", "use webcam instead of kinect").set_default(false);
};


static std::string root_string;
static std::string time_string;
static std::string color_string;
static std::string depth_string;
static std::string map_string;
static std::string ext_string;

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

int yn_func() {
    std::string yn;
    std::cin >> yn;
    if (yn == std::string("Y") || yn == std::string("y")) {
        return 1;
    } else if (yn == std::string("N") || yn == std::string("n")) {
        return -1;
    } else {
        return 0;
    }
}



void decision(void (*yf)(void), void (*nf)(void)) {
    int yn = 0;
    while (!yn) {
        yn = yn_func();
        if (yn == 1) {
            yf();
        } else if (yn == -1) {
            nf();
        }
    }
}

/***
 * 
 */
int main(int argc, char** argv) {
 
    KinectRecorderArgs args = argparse::parse<KinectRecorderArgs>(argc, argv);

    signal(SIGINT, pre_handler);

    const int fps = args.fps;
    if (fps <= 0 || fps > 30) {
        std::cout << "The framerate is invalid." << std::endl;
        std::cout << "Please choose a framerate between 1 and 30. (inclusive)" << std::endl;
        exit(-1);
    }

    root_string = args.src_path;
    time_string = root_string + std::string("/time/");
    color_string = root_string + std::string("/color/");
    depth_string = root_string + std::string("/depth/");
    map_string = root_string + std::string("/map/");
    ext_string = std::string(".bin");

    int num_frames = 0;
    const int period = (int)(1000.0 / (double)fps);

    if (fs::exists(fs::path(root_string))) {
        if (!fs::is_directory(root_string)) {
            std::cout << "Not a valid directory." << std::endl;
            exit(-1);
        }
        std::cout << "The following folder already exists:" << root_string << std::endl;
        if (!fs::is_empty(root_string)) {
            std::cout << "Would you like to delete the following subfolders? [Y/n]" << std::endl;
            std::cout << "\t- " << time_string << std::endl;
            std::cout << "\t- " << color_string << std::endl;
            std::cout << "\t- " << depth_string << std::endl;
            std::cout << "\t- " << map_string << std::endl;
            auto lambda = [](void) {
                std::cout << "Deleting folder: " << time_string << std::endl;
                fs::remove_all(time_string);
                std::cout << "Deleting folder: " << color_string << std::endl;
                fs::remove_all(color_string);
                std::cout << "Deleting folder: " << depth_string << std::endl;
                fs::remove_all(depth_string);
                std::cout << "Deleting folder: " << map_string << std::endl;
                fs::remove_all(map_string);
            };
            decision(lambda, [](void) {exit(-1);});
        }
    } else {
        std::cout << "The following folder does not exist: " << root_string << std::endl;
        std::cout << "Would you like to create this folder? [Y/n]" << std::endl;
        decision([](void) {std::cout << "Creating folder: " << root_string << std::endl;}, [](void) {exit(1);} );
        bool root_check = fs::create_directories(fs::path(root_string));
    }   

    std::cout << "Would you like to create the following folders? [Y/n]" << std::endl;
    std::cout << "\t- " << time_string << std::endl;
    std::cout << "\t- " << color_string << std::endl;
    std::cout << "\t- " << depth_string << std::endl;
    std::cout << "\t- " << map_string << std::endl;
    
    auto lambda = [](void) {
        std::cout << "Creating folder: " << time_string << std::endl;
        int time_check = mkdir(time_string.c_str(), 0777);
        std::cout << "Creating folder: " << color_string << std::endl;
        int color_check = mkdir(color_string.c_str(), 0777);
        std::cout << "Creating folder: " << depth_string << std::endl;
        int depth_check = mkdir(depth_string.c_str(), 0777);
        std::cout << "Creating folder: " << map_string << std::endl;
        int map_check = mkdir(map_string.c_str(), 0777);
    };
    decision(lambda, [](void) {exit(-1);});

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
    auto begin = std::chrono::high_resolution_clock::now();
    std::string start_filename = root_string + std::string("start.txt");
    FILE* f_start = fopen(start_filename.c_str(), "w+");
    
    while (!flag) {
        auto current_start = std::chrono::high_resolution_clock::now();
        if (!listener.waitForNewFrame(frames, 1000)) {
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
        auto current_end = std::chrono::high_resolution_clock::now();
        auto current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_end - current_start);
        while (current_duration.count() < period) {
            current_end = std::chrono::high_resolution_clock::now();
            current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_end - current_start);
        }
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
