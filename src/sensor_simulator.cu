#include "sensor_simulator.cuh"

namespace raycast
{   
    GridMap::GridMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution, int occupy_threshold = 1){
        
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        float length = max_pt(0) - min_pt(0);   // X方向的长度
        float width  = max_pt(1) - min_pt(1);   // Y方向的宽度
        float height = max_pt(2) - min_pt(2);   // Z方向的高度
        Vector3f origin(min_pt(0), min_pt(1), min_pt(2));
        Vector3f map_size(length, width, height);
        origin_x_ = origin.x;
        origin_y_ = origin.y;
        origin_z_ = origin.z;

        Vector3i grid_size;
        grid_size.x = ceil(map_size.x / resolution);
        grid_size.y = ceil(map_size.y / resolution);
        grid_size.z = ceil(map_size.z / resolution);
        int grid_total_size = grid_size.x * grid_size.y * grid_size.z;

        resolution_   = resolution;
        grid_size_x_  = grid_size.x, 
        grid_size_y_  = grid_size.y, 
        grid_size_z_  = grid_size.z, 
        grid_size_yz_ = grid_size.y * grid_size.z;
        occupy_threshold_ = occupy_threshold;
        raycast_step_ = resolution;
        int *h_map = new int[grid_total_size];
        for (int i = 0; i < grid_total_size; ++i) {
            h_map[i] = 0;
        }
        // 有时候会有全空的行，加个很小的偏移
        for (size_t i = 0; i < cloud->points.size(); i++) {
            Vector3f point(cloud->points[i].x + 0.001, cloud->points[i].y + 0.001, cloud->points[i].z + 0.001);
            int idx = Vox2Idx(Pos2Vox(point));
            if (idx < grid_total_size) {
                h_map[idx]++;
            }
        }
        cudaMalloc((void **)&map_cuda_, grid_total_size * sizeof(int));
        cudaMemcpy(map_cuda_, h_map, grid_total_size * sizeof(int), cudaMemcpyHostToDevice);
        delete[] h_map;
    }

    __host__ __device__ Vector3i GridMap::Pos2Vox(const Vector3f &pos)
    {
        Vector3i vox;
        vox.x = floor((pos.x - origin_x_) / resolution_);
        vox.y = floor((pos.y - origin_y_) / resolution_);
        vox.z = floor((pos.z - origin_z_) / resolution_);
        return vox;
    }

    __host__ __device__ Vector3f GridMap::Vox2Pos(const Vector3i &vox)
    {
        Vector3f pos;
        pos.x = (vox.x + 0.5f) * resolution_ + origin_x_;
        pos.y = (vox.y + 0.5f) * resolution_ + origin_y_;
        pos.z = (vox.z + 0.5f) * resolution_ + origin_z_;
        return pos;
    }

    __host__ __device__ int GridMap::Vox2Idx(const Vector3i &vox)
    {
        return vox.x * grid_size_yz_ + vox.y * grid_size_z_ + vox.z;
    }

    __host__ __device__ Vector3i GridMap::Idx2Vox(int idx)
    {
        return Vector3i(idx / grid_size_yz_, (idx % grid_size_yz_) / grid_size_z_, idx % grid_size_z_);
    }

    __device__ int GridMap::symmetricIndex(int index, int length)
    {
        index = index % (2 * length - 2);
        if (index < 0)
        {
            index += (2 * length - 2);
        }

        if (index >= length)
        {
            index = 2 * length - 2 - index;
        }
        return index;
    }

    // -1: z越界; 0: 空闲; 1: 占据
    __device__  int GridMap::mapQuery(const Vector3f &pos){
        Vector3i vox = Pos2Vox(pos);
        vox.x = symmetricIndex(vox.x, grid_size_x_);
        vox.y = symmetricIndex(vox.y, grid_size_y_);

        if (vox.z >= grid_size_z_)
            return 0;
        if (vox.z <= 0)
            return 1;

        int idx = Vox2Idx(vox);
        if (map_cuda_[idx] > occupy_threshold_)
            return 1;
        return 0;        
    }

    __global__ void cameraRaycastKernel(float* depth_values, GridMap grid_map, CameraParams camera_param, cudaMat::SE3<float> T_wc)
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

            // 归一化射线方向 TODO:变到世界系
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

                float3 point_c = make_float3(point_x, point_y, point_z);
                float3 point_w = T_wc * point_c;

                Vector3f point(point_w.x, point_w.y, point_w.z);

                int occupied = grid_map.mapQuery(point);

                if (occupied == 1)
                {
                    // depth = point_x;  // 直接这样赋值会有一点误差
                    // 栅格化避免平面变曲面 (有些冗余，但在机体系栅格化会有类似摩尔纹的东西)
                    Vector3i occ_vox_w = grid_map.Pos2Vox(point);
                    Vector3f occ_point_w = grid_map.Vox2Pos(occ_vox_w);
                    float3 occ_point_w_ = make_float3(occ_point_w.x, occ_point_w.y, occ_point_w.z);
                    float3 occ_point_c_ = T_wc.inv() * occ_point_w_;
                    depth = occ_point_c_.x;
                    break;
                }

                if (point_x >= camera_param.max_depth_dist){
                    depth = camera_param.max_depth_dist;
                    break;
                }
            }

            // 将深度值存储到输出数组中
            if (camera_param.normalize_depth)
                depth = depth / camera_param.max_depth_dist;
            depth_values[v * camera_param.image_width + u] = depth;
        }
    }

    cv::Mat renderDepthImage(GridMap* grid_map, CameraParams* camera_param, cudaMat::SE3<float>& T_wc)
    {   
        float* depth_values;
        size_t num_elements = camera_param->image_width * camera_param->image_height;
        cudaMallocManaged(&depth_values, num_elements * sizeof(float));

        // 在GPU上启动核函数
        cameraRaycastKernel<<<camera_param->image_height, camera_param->image_width>>>(depth_values, *grid_map, *camera_param, T_wc);
        
        cudaDeviceSynchronize();

        cv::Mat depth_image(camera_param->image_height, camera_param->image_width, CV_32FC1);

        for (int i = 0; i < camera_param->image_height; ++i) {
            for (int j = 0; j < camera_param->image_width; ++j) {
                depth_image.at<float>(i, j) = depth_values[i * camera_param->image_width + j];
            }
        }
        
        cudaFree(depth_values);
        return depth_image;
    }

    __global__ void lidarRaycastKernel(Vector3f* point_values, GridMap grid_map, LidarParams lidar_param, cudaMat::SE3<float> T_wc)
    {
        int h = threadIdx.x;
        int v = blockIdx.x;

        // printf("u: %d, v: %d \n", u, v);
        if (h < lidar_param.horizontal_num && v < lidar_param.vertical_lines)
        {   
            float vertical_resolution = (lidar_param.vertical_angle_end - lidar_param.vertical_angle_start) / (lidar_param.vertical_lines - 1);
            float vertical_angle = lidar_param.vertical_angle_start + v * vertical_resolution;
            float sin_vert = std::sin(vertical_angle * M_PI / 180.0);
            float cos_vert = std::cos(vertical_angle * M_PI / 180.0);
            float horizontal_angle = h * lidar_param.horizontal_resolution;
            float sin_horz = std::sin(horizontal_angle * M_PI / 180.0);
            float cos_horz = std::cos(horizontal_angle * M_PI / 180.0);
            // 计算射线方向
            Vector3f ray_direction(cos_vert * cos_horz, cos_vert * sin_horz, sin_vert);

            // 计算每个轴的增量比例
            float dx = ray_direction.x * grid_map.raycast_step_;
            float dy = ray_direction.y * grid_map.raycast_step_;
            float dz = ray_direction.z * grid_map.raycast_step_;

            // 递增射线方向上的每个轴
            int scale = 0;
            Vector3f point_value(0, 0, 0);

            while (1)
            {
                scale += 1;

                float point_x = scale * dx;
                float point_y = scale * dy;
                float point_z = scale * dz;

                float3 point_c = make_float3(point_x, point_y, point_z);
                float3 point_w = T_wc * point_c;

                Vector3f point(point_w.x, point_w.y, point_w.z);

                int occupied = grid_map.mapQuery(point);

                float ray_length = sqrtf(point_x * point_x + point_y * point_y + point_z * point_z);

                if (occupied == 1)
                {
                    point_value = Vector3f(point_x, point_y, point_z);
                    Vector3i vox_body = grid_map.Pos2Vox(point_value);  // 栅格化避免平面变曲面
                    point_value = grid_map.Vox2Pos(vox_body);
                    break;
                }

                if (ray_length > lidar_param.max_lidar_dist){
                    break;
                }
            }

            // 将点云值存储到输出数组中，(0, 0, 0)为无效值
            point_values[v * lidar_param.horizontal_num + h] = point_value;
        }
    }

    pcl::PointCloud<pcl::PointXYZ> renderLidarPointcloud(GridMap *grid_map, LidarParams *lidar_param, cudaMat::SE3<float>& T_wc){
        Vector3f* point_values;
        size_t num_elements = lidar_param->vertical_lines * lidar_param->horizontal_num;
        cudaMallocManaged(&point_values, num_elements * sizeof(Vector3f));

        // 在GPU上启动核函数
        lidarRaycastKernel<<<lidar_param->vertical_lines, lidar_param->horizontal_num>>>(point_values, *grid_map, *lidar_param, T_wc);
        
        cudaDeviceSynchronize();

        // NOTE: 先分配内存比push_back快很多！but这个for循环去不掉了
        pcl::PointCloud<pcl::PointXYZ> lidar_points;
        lidar_points.points.resize(num_elements);
        size_t valid_point_count = 0;
        for (int i = 0; i < lidar_param->vertical_lines; ++i) {
            for (int j = 0; j < lidar_param->horizontal_num; ++j) {
                Vector3f point = point_values[i * lidar_param->horizontal_num + j];
                if (point.x != 0 || point.y != 0 || point.z != 0)
                    lidar_points.points[valid_point_count++] = pcl::PointXYZ(point.x, point.y, point.z);
            }
        }
        lidar_points.points.resize(valid_point_count);
        
        cudaFree(point_values);
        return lidar_points;
    }


    
}