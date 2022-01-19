#include <iostream>
#include <fstream>
#include <string>
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
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda.inl.hpp>
#include <opencv2/core/cuda_types.hpp>
#include <opencv2/core/cuda_types.hpp>

#define LINE() std::cout << __LINE__ << std::endl;

#include <kinect_streamer/kinect_streamer.hpp>

struct KinectRecorderArgs : public argparse::Args {
    std::string &src_path = kwarg("d,","directory path for recorded data", " ").set_default(" ");
    std::vector<std::string> &serials = kwarg("s,serials", "Serial Numbers").multi_argument();
    bool &verbose = kwarg("v,verbose", "A flag to toggle verbose", "false").set_default(false);
};

int flag = false;

void pre_handler(int s) {
    std::cout << "Quitting!" << "\n\r";
    exit(-1);
}

void my_handler(int s) {
    std::cout << "Quitting!" << "\n\r";
    flag = true;
}

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


bool ends_with(std::string const &str, std::string const &end) {
    if (str.length() >= end.length()) {
        return (0 == str.compare(str.length() - end.length(), end.length(), end));
    } else {
        return false;
    }
}

bool starts_with(std::string const &str, std::string const &start) {
    if (str.length() >= start.length()) {
        return (0 == str.compare(0, start.length(), start));
    } else {
        return false;
    }
}

namespace fs = std::experimental::filesystem;

int main(int argc, char** argv) {
    KinectRecorderArgs args = argparse::parse<KinectRecorderArgs>(argc, argv);

    signal(SIGINT, pre_handler);

    if (!args.verbose) {
        libfreenect2::setGlobalLogger(NULL);
    }
    libfreenect2::Freenect2 freenect2;
    std::map<std::string, KinectStreamer::KinectDevice*> kin_devs;
    std::vector<std::string> serial_args = args.serials;
    std::vector<std::string> serials;
    for (std::string arg : serial_args) {
        if (is_valid_serial(arg)) {
            serials.push_back(arg);
        }
    }

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

    int n = serials.size();
    if (n > num_devices) {
        std::cout << "Too many serial numbers in input." << "\n\r";
        exit(-1);
    }

    for (std::string serial : serials) {
        KinectStreamer::KinectDevice* kin_dev = new KinectStreamer::KinectDevice(serial);
        if (!kin_dev->start()) {
            std::cout << "Failed to start Kinect Serial no.: " << serial << std::endl;
            exit(-1);
        }
        kin_dev->init_registration();
        kin_devs[serial] = kin_dev;
    }

    std::string src_path = args.src_path;
    std::cout << src_path << std::endl;

    if (src_path == " ") {

        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();

        typedef std::chrono::duration<int, std::ratio_multiply<std::chrono::hours::period, std::ratio<24>>::type> Days;

        Days days = std::chrono::duration_cast<Days>(duration);
            duration -= days;
        auto hours = std::chrono::duration_cast<std::chrono::hours>(duration);
            duration -= hours;
        auto minutes = std::chrono::duration_cast<std::chrono::minutes>(duration);
            duration -= minutes;
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
            duration -= seconds;
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
            duration -= milliseconds;
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
            duration -= microseconds;
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

        time_t rawtime;
        struct tm* ptm;
        time ( &rawtime );
        ptm = gmtime ( &rawtime );


        char datetime_buffer[64];

        std::sprintf(
            datetime_buffer,
            "%04d-%02d-%02d-%02ld-%02ld-%02ld-%03ld-%03ld-%03ld",
            (ptm->tm_year + 1900), // Year
            (ptm->tm_mon + 1), // Month
            (ptm->tm_mday), // Day
            (hours.count() + AEST) % 24, // Hour
            minutes.count(), // Minute
            seconds.count(), // Second
            milliseconds.count(), // Millisecond
            microseconds.count(), // Microsecond
            nanoseconds.count() // Nanosecond
        );
        src_path = std::string("kinect_recordings/") + datetime_buffer;
        std::cout << "No directory supplied as argument." << "\n\r";
        std::cout << "Saving in default directory: " << getenv("HOME") << "/.ros/" << src_path << "\n\r";
    }
    if (fs::exists(fs::path(src_path))) {
        if (!fs::is_directory(src_path)) {
            std::cout << "Not a valid directory." << std::endl;
            exit(-1);
        }
        if (!fs::is_empty(src_path)) {
            std::cout << "Directory is not empty." << std::endl;
            fs::remove_all(src_path);
        }
    }
    fs::create_directories(fs::path(src_path));
    std::string info_filename = src_path + "/" + "info.txt";
    std::ofstream info_file;
    info_file.open(info_filename.c_str());
    info_file << "serials:" << " ";
    for (std::string serial : serials) {
        info_file << serial << " ";
    }
    info_file << std::endl;
    info_file.close();


    for (std::string serial : serials) {
        cv::namedWindow(serial, cv::WINDOW_NORMAL);
        cv::resizeWindow(serial, cv::Size(1280, 720));
    }
    cv::namedWindow("Control Panel", cv::WINDOW_NORMAL);
    cv::resizeWindow("Control Panel", cv::Size(800, 200));

    signal(SIGINT, my_handler);
    std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> current_timestamps;
    std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> starting_timestamps;
    std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> frame_time_start;
    std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> frame_time_end;
    std::map<std::string, float> frame_rates;
    for (std::string serial : serials) {
        frame_rates[serial] = 0.0;
    }
    int take = 0;
    int frame = 0;
    bool paused = true;
    std::map<std::string, FILE*> f_colors;
    std::map<std::string, FILE*> f_depths;
    std::map<std::string, FILE*> f_times;
    for (std::string serial : serials) {
        f_colors[serial] = 0;
        f_depths[serial] = 0;
        f_times[serial] = 0;
    }
    FILE* f_depth = 0;
    FILE* f_time = 0;
    while (!flag) {
        
        for (std::string serial : serials) {
            if (!paused) {
                auto timestamp = std::chrono::high_resolution_clock::now();
                current_timestamps[serial] = timestamp;
            }
            kin_devs[serial]->wait_frames();
        }
        

        char take_buffer[128];
        std::sprintf(
            take_buffer,
            "Take No.: %02d",
            take
        );

        time_t rawtime;
        struct tm * ptm;
        time ( &rawtime );
        ptm = gmtime ( &rawtime );
        char datetime_buffer[128];
        std::sprintf(
            datetime_buffer,
            "%04d-%02d-%02d %02d:%02d:%02d",
            (ptm->tm_year + 1900), // Year
            (ptm->tm_mon + 1), // Month
            (ptm->tm_mday), // Day
            (ptm->tm_hour + AEST) % 24, // Hour
            (ptm->tm_min), // Minute
            (ptm->tm_sec) // Second
        );
        if (!paused && frame == 0) {
            info_file.open(info_filename.c_str(), std::fstream::app);
            info_file << "Take " << take << ": " << "Start Time: " << std::string(datetime_buffer) << std::endl;
            info_file.close();
        }
        int buffer = 0;

        for (std::string serial : serials) {

            libfreenect2::Frame* color = kin_devs[serial]->get_frame(libfreenect2::Frame::Color);
            libfreenect2::Frame* depth = kin_devs[serial]->get_frame(libfreenect2::Frame::Depth);

            cv::Mat img_color(cv::Size(color->width, color->height), CV_8UC4, color->data);
            cv::Mat img_depth(cv::Size(depth->width, depth->height), CV_32FC1, depth->data);
            
            cv::cvtColor(img_color, img_color, cv::COLOR_BGRA2BGR);
            cv::Mat img_display;
            cv::flip(img_color, img_display, 1);

            if (paused) {
                img_display *= 0.5;
                cv::putText(img_display, "PAUSED", cv::Point(COLOR_W / 2 - 320, COLOR_H / 2 + 20), cv::FONT_HERSHEY_PLAIN, 10, cv::Scalar(0, 0, 0), 20);
                cv::putText(img_display, "PAUSED", cv::Point(COLOR_W / 2 - 320, COLOR_H / 2 + 20), cv::FONT_HERSHEY_PLAIN, 10, cv::Scalar(255, 255, 255), 10);
            }

            int time = std::chrono::duration_cast<std::chrono::milliseconds>(current_timestamps[serial] - starting_timestamps[serial]).count();
            if (!paused) {
                fwrite(color->data, color->bytes_per_pixel, color->width * color->height, f_colors[serial]);
                fwrite(depth->data, depth->bytes_per_pixel, depth->width * depth->height, f_depths[serial]);
                fwrite(&time, sizeof(time), 1, f_times[serial]);
            }
            char stopwatch_buffer[128];
            std::sprintf(
                stopwatch_buffer,
                "%02d:%02d:%02d.%03d",
                time / 1000 / 60 / 60,
                (time / 1000 / 60) % 60,
                (time / 1000) % 60,
                time % 1000
            );

            auto timestamp = std::chrono::high_resolution_clock::now();
            frame_time_end[serial] = timestamp;


            int frame_period = std::chrono::duration_cast<std::chrono::milliseconds>(frame_time_end[serial] - frame_time_start[serial]).count();
            frame_time_start[serial] = frame_time_end[serial];
            double current_frame_rate = 1000.0 / (double)frame_period;
            frame_rates[serial] = frame_rates[serial] * 0.95 + current_frame_rate * 0.05;
            std::string frame_rate_string = std::to_string((int)round(frame_rates[serial]));
            cv::putText(img_display, stopwatch_buffer, cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 0), 5);
            cv::putText(img_display, stopwatch_buffer, cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 2);  
            cv::putText(img_display, take_buffer, cv::Point(20, 110), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 0), 5);
            cv::putText(img_display, take_buffer, cv::Point(20, 110), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 2);  
            cv::putText(img_display, std::string("FPS: ") + frame_rate_string, cv::Point(20, 160), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 0), 5);
            cv::putText(img_display, std::string("FPS: ") + frame_rate_string, cv::Point(20, 160), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 2);  
            cv::imshow(serial, img_display);
            //listeners[serial]->release(frame_maps[serial]);
            kin_devs[serial]->release_frames();

        }
        cv::Mat img_control = cv::Mat::zeros(cv::Size(800, 200), CV_8UC3);
        cv::putText(img_control, datetime_buffer, cv::Point(225, 85), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 2);
        cv::putText(img_control, take_buffer, cv::Point(225, 145), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 2);  
        if (paused) {
            cv::rectangle(img_control, cv::Rect(25, 10, 50, 180), cv::Scalar(255, 255, 255), -1, cv::LINE_8);
            cv::rectangle(img_control, cv::Rect(125, 10, 50, 180), cv::Scalar(255, 255, 255), -1, cv::LINE_8);
        } else {
            cv::Point points[1][3];
            points[0][0] = cv::Point(25, 10);
            points[0][1] = cv::Point(175, 100);
            points[0][2] = cv::Point(25, 190);
            const cv::Point* ppt[1] = { points[0] };
            int npt[] = { 3 };
            cv::fillPoly(img_control, ppt, npt, 1, cv::Scalar(255, 255, 255), cv::LINE_8);
        }
        if (!paused) {
            frame++;
        }
        cv::imshow("Control Panel", img_control);
        char c = cv::waitKey(1);
        if (c == ' ') {
            if (paused) {
                take++;
                for (std::string serial : serials) {
                    if (f_colors[serial] != 0) {
                        fclose(f_colors[serial]);
                    }
                    if (f_depths[serial] != 0) {
                        fclose(f_depths[serial]);
                    }
                    if (f_times[serial] != 0) {
                        fclose(f_times[serial]);
                    }
                    
                    std::string save_string = src_path + "/" + "takes" + "/" + std::to_string(take) + "/" + serial;
                    fs::create_directories(fs::path(save_string));
                    
                    std::string color_filename = save_string + "/" + "color.kraw";
                    std::string depth_filename = save_string + "/" + "depth.kraw";
                    std::string time_filename = save_string + "/" + "time.kraw";
                    f_colors[serial] = fopen(color_filename.c_str(), "a+");
                    f_depths[serial] = fopen(depth_filename.c_str(), "a+");
                    f_times[serial] = fopen(time_filename.c_str(), "a+");
                }
                auto timestamp = std::chrono::high_resolution_clock::now();
                for (std::string serial : serials) {
                    starting_timestamps[serial] = timestamp;
                }
                frame = 0;
            }
            paused = !paused;
        }
    }
    for (std::string serial : serials) {
        //devices[serial]->stop();
        //devices[serial]->close();
        
        kin_devs[serial]->stop();
        
        //delete devices[serial];
        //delete listeners[serial];
        //delete registrations[serial];
        delete kin_devs[serial];
    }

    return 0;
}
