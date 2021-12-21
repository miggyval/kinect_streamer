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
#define AEST (10)
#define ESC 33

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <kinect_streamer/kinect_streamer.hpp>

using namespace cv;

struct KinectRecorderArgs : public argparse::Args {
    std::string &src_path = arg("directory path for recorded data");
    int &fps = kwarg("f,framerate", "framerate (fps)").set_default(15);
    bool &webcam = flag("w,webcam", "use webcam instead of kinect").set_default(false);
    std::string &serial = kwarg("s,serial", "Serial Number").set_default("");
};


static std::string root_string;
static std::string time_string;
static std::string color_string;
static std::string depth_string;
static std::string ext_string;
static std::string timestamp_string;

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
    return 1;
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

int main(int argc, char** argv) {
 
    KinectRecorderArgs args = argparse::parse<KinectRecorderArgs>(argc, argv);
    signal(SIGINT, pre_handler);

    const int fps = args.fps;
    if (fps <= 0 || fps > FPS_MAX) {
        std::cout << "The framerate is invalid." << std::endl;
        std::cout << "Please choose a framerate between 1 and 30. (inclusive)" << std::endl;
        exit(-1);
    }
    root_string = args.src_path;
    time_string = root_string + std::string("/time/");
    color_string = root_string + std::string("/color/");
    depth_string = root_string + std::string("/depth/");
    timestamp_string = root_string + std::string("/timestamp.txt");
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
            auto lambda = [](void) {
                std::cout << "Deleting folder: " << time_string << std::endl;
                fs::remove_all(time_string);
                std::cout << "Deleting folder: " << color_string << std::endl;
                fs::remove_all(color_string);
                std::cout << "Deleting folder: " << depth_string << std::endl;
                fs::remove_all(depth_string);
            };
            //decision(lambda, [](void) {exit(-1);});
        }
    } else {
        std::cout << "The following folder does not exist: " << root_string << std::endl;
        std::cout << "Would you like to create this folder? [Y/n]" << std::endl;
        //decision([](void) {std::cout << "Creating folder: " << root_string << std::endl;}, [](void) {exit(1);} );
        bool root_check = fs::create_directories(fs::path(root_string));
    }   

    std::cout << "Would you like to create the following folders? [Y/n]" << std::endl;
    std::cout << "\t- " << time_string << std::endl;
    std::cout << "\t- " << color_string << std::endl;
    std::cout << "\t- " << depth_string << std::endl;
    
    auto lambda = [](void) {
    };

    std::cout << "Creating folder: " << time_string << std::endl;
    int time_check = mkdir(time_string.c_str(), 0777);
    std::cout << "Creating folder: " << color_string << std::endl;
    int color_check = mkdir(color_string.c_str(), 0777);
    std::cout << "Creating folder: " << depth_string << std::endl;
    int depth_check = mkdir(depth_string.c_str(), 0777);
    //decision(lambda, [](void) {exit(-1);});


    libfreenect2::setGlobalLogger(NULL);
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;
    libfreenect2::Frame undistorted(DEPTH_W, DEPTH_H, 4);
    libfreenect2::Frame registered(DEPTH_W, DEPTH_H, 4);
    libfreenect2::Registration* registration;
    std::string serial = args.serial;
    
    VideoCapture cap;

    bool use_kinect = true;
    int num_devices = freenect2.enumerateDevices();
    if (num_devices == 0) {
        std::cout << "No device connected!" << std::endl;
        std::cout << "Would you like to use a webcam? [Y/n]" << std::endl;
        //decision([](void) {}, [](void) {exit(-1);});
        std::cout << "Using webcam" << std::endl;
        use_kinect = false;
    }
    if (use_kinect) {
        if (serial == "") {
            serial = freenect2.getDefaultDeviceSerialNumber();
        }
        pipeline = new libfreenect2::OpenGLPacketPipeline();
        dev = freenect2.openDevice(serial, pipeline);
        dev->setColorFrameListener(&listener);
        dev->setIrAndDepthFrameListener(&listener);
        dev->startStreams(true, true);

        std::cout << "Device serial: " << dev->getSerialNumber() << std::endl;
        std::cout << "Device firmware: " << dev->getFirmwareVersion() << std::endl; 

        registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    } else {
        cap.open(0);
        if (!cap.isOpened()) {
            std::cout << "Unable to open camera" << std::endl;
            exit(-1);
        }
    }
    
    std::string windowName = std::string("color_") + serial;
    namedWindow(windowName, WINDOW_NORMAL);
    resizeWindow(windowName, Size(1280, 720));

    auto begin = std::chrono::high_resolution_clock::now();

    time_t rawtime;
    struct tm * ptm;
    time ( &rawtime );
    ptm = gmtime ( &rawtime );
    FILE* f_timestamp = fopen(timestamp_string.c_str(), "w+");

    fprintf(f_timestamp,
        "Start Time (AEST): %04d-%02d-%02d %02d:%02d:%2d\n",
        (ptm->tm_year + 1900), // Year
        (ptm->tm_mon + 1), // Month
        (ptm->tm_mday), // Day
        (ptm->tm_hour + AEST) % 24, // Hour
        (ptm->tm_min), // Minute
        (ptm->tm_sec) // Second
    );


    signal(SIGINT, my_handler);

    std::chrono::milliseconds pause_time = std::chrono::milliseconds(0);
    int take = 0;

    std::string take_string = std::to_string(take) + "/";
    std::string color_folder = color_string + take_string;
    std::string depth_folder = depth_string + take_string;
    std::string time_folder = time_string + take_string;
    color_check = mkdir(color_folder.c_str(), 0777);
    depth_check = mkdir(depth_folder.c_str(), 0777);
    time_check = mkdir(time_folder.c_str(), 0777);
    while (!flag) {
        auto current_start = std::chrono::high_resolution_clock::now();
        std::string color_filename = color_string + take_string + std::to_string(num_frames) + ext_string;
        std::string depth_filename = depth_string + take_string + std::to_string(num_frames) + ext_string;
        std::string time_filename = time_string + take_string + std::to_string(num_frames) + ext_string;

        FILE* f_color = fopen(color_filename.c_str(), "w+");
        FILE* f_depth = fopen(depth_filename.c_str(), "w+");
        FILE* f_time = fopen(time_filename.c_str(), "w+");

        if (!f_color || !f_depth || !f_time) {
            std::cout << color_filename << std::endl;
            std::cout << "Invalid filename" << std::endl;
            break;
        }

	    auto now = std::chrono::high_resolution_clock::now() - begin;
	    auto mill = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        fwrite(&mill, sizeof(mill), 1, f_time);
	    fclose(f_time);
        
        if (use_kinect) {
            if (!listener.waitForNewFrame(frames, 1000)) {
                std::cout << "Error!" << std::endl;
                return -1;
            }
            libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
            Mat img_color(Size(color->width, color->height), CV_8UC4, color->data);
            cvtColor(img_color, img_color, COLOR_BGRA2BGR);
            Mat img_display;
            cv::flip(img_color, img_display, 1);
            imshow(windowName, img_display);
            char c = waitKey(1);
            if (c == ' ' || c == 'p') {
                auto start_pause = std::chrono::high_resolution_clock::now();
                take++;
                take_string = std::to_string(take) + "/";
                color_folder = color_string + take_string;
                depth_folder = depth_string + take_string;
                time_folder = time_string + take_string;
                color_check = mkdir(color_folder.c_str(), 0777);
                depth_check = mkdir(depth_folder.c_str(), 0777);
                time_check = mkdir(time_folder.c_str(), 0777);
                c = waitKey(1);
                while (c != ' ' && c != 'p') {
                    if (c == 'q' || c == ESC) {
                        std::cout << "Quitting!" << std::endl;
                        flag = true;
                        break;
                    }
                    // PAUSED!

                    //
                    Mat img_paused = img_color * 0.3;
                    cv::flip(img_paused, img_paused, 1);
                    putText(img_paused, "PAUSED", Point(COLOR_W / 2 - 128, COLOR_H / 2 - 10), FONT_HERSHEY_PLAIN,  5, Scalar(128, 128, 128), 2);
                    putText(img_paused, (std::string("Take No: ") + std::to_string(take)).c_str(), Point(COLOR_W / 2 - 200, COLOR_H / 2 + 120), FONT_HERSHEY_PLAIN,  5, Scalar(0, 0, 255), 2);
                    imshow(windowName, img_paused);
                    c = waitKey(1);
                }
                auto end_pause = std::chrono::high_resolution_clock::now();
                auto duration_pause = std::chrono::duration_cast<std::chrono::milliseconds>(end_pause - start_pause);
                pause_time += duration_pause;
            } else if (c == 'q' || c == ESC) {
                flag = true;
                std::cout << "Quitting!" << std::endl;
            }
            fwrite(color->data, color->bytes_per_pixel, color->width * color->height, f_color);
            fwrite(depth->data, depth->bytes_per_pixel, depth->width * depth->height, f_depth);

            listener.release(frames);

            fclose(f_color);
            fclose(f_depth);
            
        } else {
            Mat img_color;
            cap.read(img_color);
            if (img_color.empty()) {
                std::cout << "Error!" << std::endl;
                return -1;
            }
            Mat img_converted;
            cvtColor(img_color, img_converted, COLOR_BGR2BGRA);
            Mat img_resized;
            resize(img_converted, img_resized, Size(COLOR_W, COLOR_H));
            Mat img_depth = Mat::zeros(Size(DEPTH_W, DEPTH_H), CV_32FC1);
            cv::Mat img_display;
            cv::flip(img_resized, img_display, 1);
            imshow(windowName, img_display);
            char c = waitKey(1);
            if (c == ' ' || c == 'p') {
                auto start_pause = std::chrono::high_resolution_clock::now();
                
                c = waitKey(1);
                while (c != ' ' && c != 'p') {
                    if (c == 'q' || c == ESC) {
                        std::cout << "Quitting!" << std::endl;
                        flag = true;
                        break;
                    }
                    Mat img_paused = img_resized * 0.3;
                    putText(img_paused, "PAUSED", Point(COLOR_W / 2 - 128, COLOR_H / 2 - 10), FONT_HERSHEY_PLAIN,  5, Scalar(128, 128, 128), 2);
                    imshow(windowName, img_paused);
                    c = waitKey(1);
                }
                auto end_pause = std::chrono::high_resolution_clock::now();
                auto duration_pause = std::chrono::duration_cast<std::chrono::milliseconds>(end_pause - start_pause);
                pause_time += duration_pause;
            } else if (c == 'q' || c == ESC) {
                flag = true;
                std::cout << "Quitting!" << std::endl;
            }
            fwrite(img_resized.data, img_resized.elemSize(), img_resized.cols * img_resized.rows, f_color);
            fwrite(img_depth.data, img_depth.elemSize(), img_depth.cols * img_resized.rows, f_depth);
            fclose(f_color);
            fclose(f_depth);
        }
        num_frames++;
        auto current_end = std::chrono::high_resolution_clock::now();
        auto current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_end - current_start);
        while (current_duration.count() < period) {
            current_end = std::chrono::high_resolution_clock::now();
            current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_end - current_start);
        }
    }
    auto end = std::chrono::high_resolution_clock::now();    
    auto duration = end - begin - pause_time;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    std::cout << "Time = " << ms << " ms" << std::endl;
    std::cout << "Frames = " << num_frames << std::endl;
    std::cout << "Framerate = " << (double)num_frames / ((double)ms / 1000.0) << " FPS" << std::endl;
    if (use_kinect) {
        dev->stop();
        dev->close();
    }
    return 0;
}
