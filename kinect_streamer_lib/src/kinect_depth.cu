#include <stdio.h>
#include <cuda_runtime.h>
#include <kinect_depth/kinect_depth.h>

__global__ void getPointXYZ(const float* D, const uint32_t* R, uint8_t* cloud_data, float cx, float cy, float fx, float fy, int width, int height) {
    int numElements = width * height;
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i < numElements) {
        int row = i / width;
        int col = i % width;
        const float depth_val = D[width * row + col] / 1000.0f;
        if (!isnan(depth_val) && depth_val > 0.001) {
            /*
            pcl::PointXYZRGB point;
            point[0] = -(col + 0.5 - cx) * fx * depth_val;
            point.y = (row + 0.5 - cy) * fy * depth_val;
            point.z = depth_val;
            const uint8_t *p = reinterpret_cast<uint8_t*>(R[512 * row + col]);
            point.b = p[0];
            point.g = p[1];
            point.r = p[2];
            */

        }
    }
}

void getPointXYZHelper(const float* D, const uint32_t* R, uint8_t* cloud_data, float cx, float cy, float fx, float fy, int width, int height) {
    int numElements = width * height;
    int threadsPerBlock = 1024;
    int blocksPerGrid = (numElements + threadsPerBlock - 1) / threadsPerBlock;
    getPointXYZ<<<blocksPerGrid, threadsPerBlock>>>(D, R, cloud_data, cx, cy, fx, fy, width, height);
}