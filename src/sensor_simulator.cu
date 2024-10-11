#include "sensor_simulator.cuh"

namespace raycast
{
    __device__ float resolution_;
    __device__ float origin_x_, origin_y_, origin_z_;
    __device__ int grid_size_yz_, grid_size_z_;
    int *map_cuda_;

    __global__ void init_params(float resolution, float origin_x, float origin_y, float origin_z,
                                int grid_size_yz, int grid_size_z)
    {
        resolution_ = resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        origin_z_ = origin_z;
        grid_size_yz_ = grid_size_yz;
        grid_size_z_ = grid_size_z;
    }

    __device__ Vector3i Pos2Vox(const Vector3d &pos)
    {
        Vector3i vox;
        vox.x = floor((pos.x - origin_x_) / resolution_);
        vox.y = floor((pos.y - origin_y_) / resolution_);
        vox.z = floor((pos.z - origin_z_) / resolution_);
        return vox;
    }

    __device__ Vector3d Vox2Pos(const Vector3i &vox)
    {
        Vector3d pos;
        pos.x = (vox.x + 0.5f) * resolution_ + origin_x_;
        pos.y = (vox.y + 0.5f) * resolution_ + origin_y_;
        pos.z = (vox.z + 0.5f) * resolution_ + origin_z_;
        return pos;
    }

    __device__ int Vox2Idx(const Vector3i &vox)
    {
        return vox.x * grid_size_yz_ + vox.y * grid_size_z_ + vox.z;
    }

    __device__ Vector3i Idx2Vox(int idx)
    {
        return Vector3i(idx / grid_size_yz_, (idx % grid_size_yz_) / grid_size_z_, idx % grid_size_z_);
    }

    __device__ int Pos2Idx(const Vector3d &pos)
    {
        return Vox2Idx(Pos2Vox(pos));
    }

    void ConstructMap(Vector3d origin, double resolution, Vector3d map_size)
    {
        Vector3i grid_size;
        grid_size.x = ceil(map_size.x / resolution);
        grid_size.y = ceil(map_size.y / resolution);
        grid_size.z = ceil(map_size.z / resolution);

        int grid_size_z = grid_size.z;
        int grid_size_yz = grid_size.y * grid_size.z;
        int grid_total_size = grid_size.x * grid_size.y * grid_size.z;

        init_params<<<1, 1>>>(resolution, origin.x, origin.y, origin.z, grid_size_yz, grid_size_z);
        cudaDeviceSynchronize();

        bool *h_dynamicArray = new bool[grid_total_size];
        for (int i = 0; i < grid_total_size; ++i)
        {
            h_dynamicArray[i] = false;
        }
        // 地图赋值

        cudaMalloc((void **)&map_cuda_, grid_total_size * sizeof(int));
        cudaMemcpy(map_cuda_, h_dynamicArray, grid_total_size * sizeof(int), cudaMemcpyHostToDevice);

        delete[] h_dynamicArray;
    }

    // 核函数定义
    __global__ void testPos2IdxKernel(Vector3d *positions, int *indices)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x; // CUDA内核的线程索引
        indices[idx] = Pos2Idx(positions[idx]);          // 访问设备端的 RayCast 实例
    }

    void CalculateVox2Idx()
    {
        // 创建输入位置数组
        int count = 100;             // 运行次数
        Vector3d h_positions[count]; // 主机上的位置数组
        Vector3d *d_positions;       // 设备上的位置数组
        int h_indices[count];        // 主机上的索引数组
        int *d_indices;              // 设备上的索引数组

        for (int i = 0; i < count; i++)
        {
            h_positions[i].x = i;
        }

        // 分配设备内存
        cudaMalloc((void **)&d_positions, count * sizeof(Vector3d));
        cudaMalloc((void **)&d_indices, count * sizeof(int));

        // 将位置数组从主机复制到设备
        cudaMemcpy(d_positions, h_positions, count * sizeof(Vector3d), cudaMemcpyHostToDevice);

        // 启动CUDA内核
        int threadsPerBlock = 32;
        int blocksPerGrid = (count + threadsPerBlock - 1) / threadsPerBlock;
        testPos2IdxKernel<<<blocksPerGrid, threadsPerBlock>>>(d_positions, d_indices);

        // 将结果从设备复制回主机
        cudaMemcpy(h_indices, d_indices, count * sizeof(int), cudaMemcpyDeviceToHost);

        // 输出结果
        for (int i = 0; i < count; ++i)
        {
            std::cout << "Position: (" << h_positions[i].x << ", " << h_positions[i].y << ", " << h_positions[i].z << ") -> Index: " << h_indices[i] << std::endl;
        }

        // 释放设备内存
        cudaFree(d_positions);
        cudaFree(d_indices);
    }

}