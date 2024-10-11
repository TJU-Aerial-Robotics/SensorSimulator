#include "sensor_simulator.cuh"

namespace raycast
{   
    GridMap::GridMap(Vector3d origin, double resolution, Vector3d map_size){
        // 改成地图在外面创建好，直接传进来
        origin_x_ = origin.x;
        origin_y_ = origin.y;
        origin_z_ = origin.z;

        Vector3i grid_size;
        grid_size.x = ceil(map_size.x / resolution);
        grid_size.y = ceil(map_size.y / resolution);
        grid_size.z = ceil(map_size.z / resolution);

        int grid_total_size = grid_size.x * grid_size.y * grid_size.z;

        resolution_ = resolution;
        grid_size_x_ = grid_size.x, 
        grid_size_y_ = grid_size.y, 
        grid_size_z_ = grid_size.z, 
        grid_size_yz_ = grid_size.y * grid_size.z;
        occupy_threshold = 1;
        raycast_step_ = resolution;

        int *h_map = new int[grid_total_size];
        for (int i = 0; i < grid_total_size; ++i)
        {
            h_map[i] = 0;
        }
        // 地图赋值

        cudaMalloc((void **)&map_cuda_, grid_total_size * sizeof(int));
        cudaMemcpy(map_cuda_, h_map, grid_total_size * sizeof(int), cudaMemcpyHostToDevice);

        delete[] h_map;
    }

    __device__ Vector3i GridMap::Pos2Vox(const Vector3d &pos)
    {
        Vector3i vox;
        vox.x = floor((pos.x - origin_x_) / resolution_);
        vox.y = floor((pos.y - origin_y_) / resolution_);
        vox.z = floor((pos.z - origin_z_) / resolution_);
        return vox;
    }

    __device__ Vector3d GridMap::Vox2Pos(const Vector3i &vox)
    {
        Vector3d pos;
        pos.x = (vox.x + 0.5f) * resolution_ + origin_x_;
        pos.y = (vox.y + 0.5f) * resolution_ + origin_y_;
        pos.z = (vox.z + 0.5f) * resolution_ + origin_z_;
        return pos;
    }

    __device__ int GridMap::Vox2Idx(const Vector3i &vox)
    {
        return vox.x * grid_size_yz_ + vox.y * grid_size_z_ + vox.z;
    }

    __device__ Vector3i GridMap::Idx2Vox(int idx)
    {
        return Vector3i(idx / grid_size_yz_, (idx % grid_size_yz_) / grid_size_z_, idx % grid_size_z_);
    }

    __device__  bool GridMap::mapQuery(const Vector3d &pos){
        Vector3i vox = Pos2Vox(pos);
        while (vox.x > grid_size_x_)
            vox.x -= grid_size_x_;
        while (vox.y > grid_size_y_)
            vox.y -= grid_size_y_;
        while (vox.z > grid_size_z_)
            vox.z -= grid_size_z_;
        while (vox.x < 0)
            vox.x += grid_size_x_;
        while (vox.y < 0)
            vox.y += grid_size_y_;
        while (vox.z < 0)
            vox.z += grid_size_z_;
        int idx = Vox2Idx(vox);

        return map_cuda_[idx] > occupy_threshold;
    }

    __global__ void raycastKernel(float* depth_values, GridMap grid_map, CameraParams camera_param)
    {
        int u = threadIdx.x;
        int v = blockIdx.x;

        // printf("u: %d, v: %d \n", u, v);

        if (u < camera_param.image_width && v < camera_param.image_height)
        {
            // 计算射线方向
            float y = -(u - camera_param.cx) / camera_param.fx;
            float z = -(v - camera_param.cy) / camera_param.fy;
            float x = 1.0f;

            // 归一化射线方向
            float length = sqrtf(x * x + y * y + z * z);
            x /= length;
            y /= length;
            z /= length;

            // 计算每个轴的增量比例
            float dx = x * grid_map.raycast_step_;
            float dy = y * grid_map.raycast_step_;
            float dz = z * grid_map.raycast_step_;

            // 递增射线方向上的每个轴
            int scale = 0;
            float depth = 0.0f;

            while (1)
            {
                scale += 1;

                float point_x = scale * dx;
                float point_y = scale * dy;
                float point_z = scale * dz;
                Vector3d point(point_x, point_y, point_z);

                bool occupied = grid_map.mapQuery(point);

                float ray_length = sqrtf(point_x * point_x + point_y * point_y + point_z * point_z);
                if (occupied || ray_length > camera_param.max_depth_dist){
                    depth = ray_length;
                    break;
                }
            }

            // 将深度值存储到输出数组中
            depth_values[v * camera_param.image_width + u] = depth;
        }
    }

    cv::Mat rayCast(GridMap* grid_map, CameraParams* camera_param)
    {   
        cudaEvent_t start, stop;
        cudaEventCreate(&start);
        cudaEventCreate(&stop);
        cudaEventRecord(start);

        // 分配内存来存储射线方向
        float* depth_values;
        size_t num_elements = camera_param->image_width * camera_param->image_height;
        cudaMallocManaged(&depth_values, num_elements * sizeof(float));

        // 在GPU上启动核函数
        raycastKernel<<<camera_param->image_height, camera_param->image_width>>>(depth_values, *grid_map, *camera_param);
        
        // 同步线程，确保所有计算完成
        cudaDeviceSynchronize();

        cudaEventRecord(stop);
        cudaEventSynchronize(stop);
        float milliseconds = 0;
        cudaEventElapsedTime(&milliseconds, start, stop);
        std::cout << "rayCast execution time: " << milliseconds << " ms" << std::endl;

        cv::Mat depth_image(camera_param->image_height, camera_param->image_width, CV_32FC1);

        // 将 depth_values 数据复制到 Mat 中
        for (int i = 0; i < camera_param->image_height; ++i) {
            for (int j = 0; j < camera_param->image_width; ++j) {
                depth_image.at<float>(i, j) = depth_values[i * camera_param->image_width + j];
            }
        }
        
        cudaFree(depth_values);

        return depth_image;
    }

}