#include <kinect_streamer/kinect_streamer.hpp>


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


using namespace cv;

int main(int argc, char** argv) {
    std::cout << std::endl;
    KinectStreamer::KinectDevice kinect_dev;
    kinect_dev.start(); 
    namedWindow("Hello", WINDOW_NORMAL);
    resizeWindow("Hello", Size(1280, 720));
    while (1) {
        libfreenect2::Frame* color = kinect_dev.get_frame(libfreenect2::Frame::Color);
        Mat img_color(Size(color->width, color->height), CV_8UC4, color->data);
        cvtColor(img_color, img_color, COLOR_BGRA2BGR);
        imshow("Hello", img_color);
        waitKey(1);
        kinect_dev.release_frames();
    }
    return 0;
}