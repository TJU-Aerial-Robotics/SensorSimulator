#include <iostream>
#include "sensor_simulator.cuh"

using namespace raycast;

int main() {
    // 参数设置
    float resolution = 0.2f; // 分辨率
    Vector3d origin(0.0f, 0.0f, 0.0f); // 原点
    Vector3d map_size(10, 10, 10); // 原点
    GridMap* grid_map = new GridMap(origin, resolution, map_size);
    CameraParams* camera = new CameraParams();

    cv::Mat depth_image;
    depth_image = rayCast(grid_map, camera);

    return 0;
}
