#include <iostream>
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

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>


using namespace cv;

#define MODE_DIR    1
#define MODE_FILE   2

struct KinectViewerArgs : public argparse::Args {
    std::string &src_dir_path = kwarg("d,dir", "directory path for recorded data").set_default("");
    std::string &src_file_path = kwarg("f,file", "file path for recorded data").set_default("");
};

namespace fs = std::experimental::filesystem;
const int color_size = COLOR_W * COLOR_H;
const int depth_size = DEPTH_W * DEPTH_H;

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
        return (0 == str.compare (str.length() - end.length(), end.length(), end));
    } else {
        return false;
    }
}


int main(int argc, char** argv) {

    KinectViewerArgs args = argparse::parse<KinectViewerArgs>(argc, argv);

    std::vector<std::string> serial_args;
    std::vector<std::string> serials;
    std::string info_filename = "";
    int mode = 0;
    if (args.src_dir_path == "" && args.src_file_path != "") {
        std::string parent_path = "";
        if (ends_with(args.src_file_path, "color.kraw")) {
            parent_path = args.src_file_path.substr(0, args.src_file_path.find("color.kraw"));
        }
        if (ends_with(args.src_file_path, "depth.kraw")) {
            parent_path = args.src_file_path.substr(0, args.src_file_path.find("depth.kraw"));
        }
        std::cout << "Parent Path: " << parent_path << "\n\r";
        info_filename = parent_path + "/../../../info.txt";
        mode = MODE_FILE;
    } else if (args.src_dir_path != "" && args.src_file_path == "") {
        info_filename = args.src_dir_path + "/" + "info.txt";
        mode = MODE_DIR;
    } else {
        std::cout << "Please enter a filename or directory name." << "\n\r";
    }
    FILE* f_info = fopen(info_filename.c_str(), "r");
    std::cout << info_filename << std::endl;
    if (!f_info) {
        std::cout << "Info file (info.txt) is missing." << "\n\r";
        return -1;
    }
    char info_str[128];
    char dummy[128];
    int take_count = 0;
    if (fgets(info_str, 128, f_info) == NULL) {
        std::cout << "Failed to read info.txt" << std::endl;
    }
    while (fgets(dummy, 128, f_info) != NULL) {
        take_count++;
    }
    std::string info_string(info_str);
    std::string delimiter = " ";

    size_t pos = 0;
    std::string token;
    while ((pos = info_string.find(delimiter)) != std::string::npos) {
        token = info_string.substr(0, pos);
        serial_args.push_back(token);
        info_string.erase(0, pos + delimiter.length());
    }
    for (std::string arg : serial_args) {
        if (is_valid_serial(arg)) {
            serials.push_back(arg);
        }
    }
    if (mode == MODE_DIR) {
    for (std::string serial : serials) {
        namedWindow(serial + "_color", WINDOW_NORMAL);
        namedWindow(serial + "_depth", WINDOW_NORMAL);
        resizeWindow(serial + "_color", Size(1920 / 2, 1080 / 2));
    }
    } else if (mode == MODE_FILE) {
        if (ends_with(args.src_file_path, "color.kraw")) {
            namedWindow("color", WINDOW_NORMAL);
            resizeWindow("color", Size(1920 / 2, 1080 / 2));
        } else if (ends_with(args.src_file_path, "depth.kraw")) {
            namedWindow("depth", WINDOW_NORMAL);
        }
    }
    unsigned int time = 0;
    if (mode == MODE_DIR) {
        int start_take = 1;
        int end_take = take_count;
        for (int take = start_take; take <= end_take; take++) {
            waitKey(0);
            std::cout << "Playing Take No.: " << take << std::endl; 
            std::map<std::string, FILE*> f_colors;
            std::map<std::string, FILE*> f_depths;
            std::map<std::string, FILE*> f_times;
            for (std::string serial : serials) {
                std::string save_string = args.src_dir_path + "/" + "takes" + "/" + std::to_string(take) + "/" + serial;
                std::string color_filename = save_string + "/" + "color" + ".kraw";
                std::string depth_filename = save_string + "/" + "depth" + ".kraw";
                std::string time_filename = save_string + "/" + "time" + ".kraw";           

                f_colors[serial] = fopen(color_filename.c_str(), "r");
                f_depths[serial] = fopen(depth_filename.c_str(), "r");
                f_times[serial] = fopen(time_filename.c_str(), "r");
            }
            std::map<std::string, bool> finished_map;
            for (std::string serial : serials) {
                finished_map[serial] = false;
            }
            std::map<std::string, Mat> color_imgs;
            std::map<std::string, Mat> depth_imgs;

            for (std::string serial : serials) {
                color_imgs[serial] = Mat(Size(COLOR_W, COLOR_H), CV_8UC4);
                depth_imgs[serial] = Mat(Size(DEPTH_W, DEPTH_H), CV_32FC1);
            
            }
            bool finished = false;
            while (!finished) {
                for (std::string serial : serials) {
                    int color_bytes_read = fread(color_imgs[serial].data, color_imgs[serial].elemSize(), color_size, f_colors[serial]);
                    int depth_bytes_read = fread(depth_imgs[serial].data, depth_imgs[serial].elemSize(), depth_size, f_depths[serial]);
                    int time_bytes_read = fread(&time, sizeof(unsigned int), 1, f_times[serial]);
                    if (!color_bytes_read || !depth_bytes_read || !time_bytes_read) {
                        finished_map[serial] = true;
                        continue;
                    }
                    depth_imgs[serial] /= DEPTH_MAX;
                    imshow(serial + "_color", color_imgs[serial]);
                    imshow(serial + "_depth", depth_imgs[serial]);
                }
                waitKey(16);
                finished = true;
                for (std::string serial : serials) {
                    finished = finished && finished_map[serial];
                }
            }
            for (std::string serial : serials) {
                fclose(f_colors[serial]);
                fclose(f_depths[serial]);
                fclose(f_times[serial]);
            }
        } 
    } else if (mode == MODE_FILE) {
        if (ends_with(args.src_file_path, "color.kraw")) {
            FILE* f_color = fopen(args.src_file_path.c_str(), "r");
            while (true) {
                Mat color_img = Mat(Size(COLOR_W, COLOR_H), CV_8UC4);
                int color_bytes_read = fread(color_img.data, color_img.elemSize(), color_size, f_color);
                if (!color_bytes_read) {
                    break;
                }
                imshow("color", color_img);
                waitKey(16);
            }
            fclose(f_color);
        } else if (ends_with(args.src_file_path, "depth.kraw")) {
            FILE* f_depth = fopen(args.src_file_path.c_str(), "r");
            while (true) {
                Mat depth_img = Mat(Size(DEPTH_W, DEPTH_H), CV_32FC1);
                int depth_bytes_read = fread(depth_img.data, depth_img.elemSize(), depth_size, f_depth);
                if (!depth_bytes_read) {
                    break;
                }
                imshow("depth", depth_img / DEPTH_MAX);
                waitKey(16);
            }
            fclose(f_depth);
        }
    }
    fclose(f_info);
}