#include <iostream>
#include "sensor_simulator.cuh"

using namespace raycast;

int main() {
    // 参数设置
    float resolution = 1.0f; // 分辨率
    Vector3d origin(0.0f, 0.0f, 0.0f); // 原点
    int grid_size_yz = 20; // 网格尺寸
    int grid_size_z = 10; // 网格尺寸
    CalculateVox2Idx();

    return 0;
}
