#ifndef CUDA_UTILS_CUH
#define CUDA_UTILS_CUH

#include <cuda_runtime.h>
#include <cmath>
#include <iostream>

namespace raycast
{

    struct Vector3d
    {
        float x, y, z;
        __device__ __host__ Vector3d() : x(0.0f), y(0.0f), z(0.0f) {}
        __device__ __host__ Vector3d(float x_val, float y_val, float z_val)
            : x(x_val), y(y_val), z(z_val) {}
    };

    struct Vector3i
    {
        int x, y, z;
        __device__ __host__ Vector3i() : x(0), y(0), z(0) {}
        __device__ __host__ Vector3i(int x_val, int y_val, int z_val)
            : x(x_val), y(y_val), z(z_val) {}
    };

    __device__ Vector3i Pos2Vox(const Vector3d &pos);
    __device__ Vector3d Vox2Pos(const Vector3i &vox);
    __device__ int Vox2Idx(const Vector3i &vox);
    __device__ Vector3i Idx2Vox(int idx);
    __device__ int Pos2Idx(const Vector3d &pos);

    __global__ void init_params();
    __global__ void testPos2IdxKernel(Vector3d *positions, int *indices);

    void launch_init_params();

    void CalculateVox2Idx();

}
#endif // CUDA_UTILS_CUH
