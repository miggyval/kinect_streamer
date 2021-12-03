#ifndef KINECT_DEPTH_H
#define KINECT_DEPTH_H

void getPointXYZHelper(const float* D, float* X, float* Y, float* Z, float cx, float cy, float fx, float fy, int width, int height);

#endif