#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <cuda_runtime.h>
#include <kinect_depth/kinect_depth.h>

__global__ void getPointXYZ(const float* D, const uint32_t* R, uint8_t* cloud_data, float cx, float cy, float fx, float fy, int width, int height) {
    int numElements = width * height;
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    const int point_step = 32;
    if (i < numElements) {
        int row = i / width;
        int col = i % width;
        const float depth_val = D[width * row + col] / 1000.0f;
        if (!isnan(depth_val) && depth_val > 0.001) {
            uint8_t* ptr = cloud_data + i * point_step;
            /* x-value */
            *(float*)(ptr + 0) = -(col + 0.5 - cx) * fx * depth_val;
            /* y-value */
            *(float*)(ptr + 4) = (row + 0.5 - cy) * fy * depth_val;
            /* z-value */
            *(float*)(ptr + 8) = depth_val;
            /* rgb-value */
            *(uint32_t*)(ptr + 16) = R[i];
        }
    }
}

void getPointXYZHelper(const float* D, const uint32_t* R, uint8_t* cloud_data, float cx, float cy, float fx, float fy, int width, int height) {
    int numElements = width * height;
    int threadsPerBlock = 1024;
    int blocksPerGrid = (numElements + threadsPerBlock - 1) / threadsPerBlock;
    getPointXYZ<<<blocksPerGrid, threadsPerBlock>>>(D, R, cloud_data, cx, cy, fx, fy, width, height);
}