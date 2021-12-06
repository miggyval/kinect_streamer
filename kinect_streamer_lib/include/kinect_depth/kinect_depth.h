#ifndef KINECT_DEPTH_H
#define KINECT_DEPTH_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

void getPointXYZHelper(const float* D, const uint32_t* R, uint8_t* cloud_data, float cx, float cy, float fx, float fy, int width, int height);

#endif