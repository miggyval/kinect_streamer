#include <stdio.h>
#include <cuda_runtime.h>
#include <kinect_depth/kinect_depth.h>

__global__ void getPointXYZ(const float* D, float* X, float* Y, float* Z, float cx, float cy, float fx, float fy, int width, int height) {
    int numElements = width * height;
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i < numElements) {
        int row = i / width;
        int col = i % width;
        const float depth_val = D[width * row + col] / 1000.0f;
        if (!isnan(depth_val) && depth_val > 0.001) {
            X[i] = -(col + 0.5 - cx) * fx * depth_val;
            Y[i] = (row + 0.5 - cy) * fy * depth_val;
            Z[i] = depth_val;
        }
    }
}

void getPointXYZHelper(const float* D, float* X, float* Y, float* Z, float cx, float cy, float fx, float fy, int width, int height) {
    int numElements = width * height;
    int threadsPerBlock = 1024;
    int blocksPerGrid = (numElements + threadsPerBlock - 1) / threadsPerBlock;
    getPointXYZ<<<blocksPerGrid, threadsPerBlock>>>(D, X, Y, Z, cx, cy, fx, fy, width, height);
}