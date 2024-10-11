#ifndef CUDA_UTILS_CUH
#define CUDA_UTILS_CUH

#include <cuda_runtime.h>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

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

    struct CameraParams
    {
        float fx = 80.0f; // focal length x
        float fy = 80.0f; // focal length y
        float cx = 80.0f; // principal point x (image center)
        float cy = 45.0f; // principal point y (image center)
        int image_width = 160;
        int image_height = 90;
        float max_depth_dist{20};
        bool normalize_depth{false};
    };

    struct LidarParams
    {
        int vertical_lines = 16;            // 纵向16线
        float vertical_angle_start = -15.0; // 起始垂直角度
        float vertical_angle_end = 15.0;    // 结束垂直角度
        int horizontal_angle = 360;         // 水平360线
        float horizontal_resolution = 1.0;  // 水平分辨率为1度
        float max_lidar_dist{50};
    };

    class GridMap
    {
        public:
            GridMap(Vector3d origin, double resolution, Vector3d map_size);
            ~GridMap() { cudaFree(map_cuda_); };
            __device__ Vector3i Pos2Vox(const Vector3d &pos);
            __device__ Vector3d Vox2Pos(const Vector3i &vox);
            __device__ int Vox2Idx(const Vector3i &vox);
            __device__ Vector3i Idx2Vox(int idx);
            __device__ bool mapQuery(const Vector3d &pos);

            float raycast_step_; // raycast step
        private:
            // map param
            int *map_cuda_;
            float resolution_;                                           // grid resolution
            float origin_x_, origin_y_, origin_z_;                       // origin coordinates
            int grid_size_x_, grid_size_y_, grid_size_z_, grid_size_yz_; // grid sizes
            int occupy_threshold;                                        // occupancy threshold
    };

    __global__ void raycastKernel(float *depth_values, GridMap grid_map, CameraParams camera_param);

    cv::Mat rayCast(GridMap *grid_map, CameraParams *camera_param);

}
#endif // CUDA_UTILS_CUH
