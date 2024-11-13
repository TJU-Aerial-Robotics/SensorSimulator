#ifndef CUDA_UTILS_CUH
#define CUDA_UTILS_CUH

#include <cuda_runtime.h>
#include "cuda_toolkit/se3.cuh"
#include <cmath>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h> // For pcl::getMinMax3D
#include <pcl/point_cloud.h>   // For pcl::PointCloud
#include <pcl/point_types.h>   // For pcl::PointXYZ
#include <chrono>

namespace raycast
{
    struct Vector3f
    {
        float x, y, z;
        __device__ __host__ Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
        __device__ __host__ Vector3f(float x_val, float y_val, float z_val)
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
        int horizontal_num = 360;           // 水平360点
        float horizontal_resolution = 1.0;  // 水平分辨率为1度
        float max_lidar_dist{50};
    };

    class GridMap
    {
    public:
        GridMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution, int occupy_threshold);
        ~GridMap() {};
        void freeGridMap() { cudaFree(map_cuda_); }
        __host__ __device__ Vector3i Pos2Vox(const Vector3f &pos);
        __host__ __device__ Vector3f Vox2Pos(const Vector3i &vox);
        __host__ __device__ int Vox2Idx(const Vector3i &vox);
        __host__ __device__ Vector3i Idx2Vox(int idx);
        __device__ int symmetricIndex(int index, int length);
        __device__ int mapQuery(const Vector3f &pos);

        float raycast_step_; // raycast step
    private:
        // map param
        int *map_cuda_;
        float resolution_;                                           // grid resolution
        float origin_x_, origin_y_, origin_z_;                       // origin coordinates
        int grid_size_x_, grid_size_y_, grid_size_z_, grid_size_yz_; // grid sizes
        int occupy_threshold_;                                       // occupancy threshold
    };

    __global__ void cameraRaycastKernel(float *depth_values, GridMap grid_map, CameraParams camera_param, cudaMat::SE3<float> T_wc);
    __global__ void lidarRaycastKernel(Vector3f *point_values, GridMap grid_map, LidarParams lidar_param, cudaMat::SE3<float> T_wc);

    cv::Mat renderDepthImage(GridMap *grid_map, CameraParams *camera_param, cudaMat::SE3<float> &T_wc);
    pcl::PointCloud<pcl::PointXYZ> renderLidarPointcloud(GridMap *grid_map, LidarParams *lidar_param, cudaMat::SE3<float> &T_wc);
}
#endif // CUDA_UTILS_CUH
