#include "sensor_simulator.h"

cv::Mat SensorSimulator::renderDepthImage(){

    cv::Mat depth_image(image_height, image_width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));
    Eigen::Matrix3f R_wc = quat.toRotationMatrix();
    Eigen::Matrix3f R_cw = R_wc.inverse();

    auto start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
    for (int v = 0; v < image_height; ++v) {
        for (int u = 0; u < image_width; ++u) {
            // 计算射线方向（图像平面坐标系）
            float y = -(u - cx) / fx;
            float z = -(v - cy) / fy;
            float x = 1.0f;
            Eigen::Vector3f d(x, y, z);
            d.normalize();

            // 转换到世界坐标系下
            Eigen::Vector3f ray_direction = R_wc * d;  // 考虑相机旋转
            Eigen::Vector3f ray_origin = pos;         // 相机的位置

            // 使用Octree查找射线方向上最近的点
            std::vector<int> pointIdxVec;
            if (octree->getIntersectedVoxelIndices(ray_origin, ray_direction, pointIdxVec, 1)) {
                pcl::PointXYZ closest_point = cloud->points[pointIdxVec[0]];
                Eigen::Vector3f point_in_world = closest_point.getVector3fMap();
                Eigen::Vector3f closest_point_camera = R_cw * (point_in_world - pos);
                float distance = closest_point_camera(0);
                if (distance < 0) distance = 0;
                if (distance > max_depth_dist) distance = max_depth_dist;
                if (normalize_depth) distance = distance / max_depth_dist;
                depth_image.at<float>(v, u) = distance;
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    // std::cout << "生成图像耗时: " << elapsed.count() << " 秒" << std::endl; // 输出耗时

    // 将无效值设置为0
    for (int v = 0; v < image_height; ++v) {
        for (int u = 0; u < image_width; ++u) {
            if (depth_image.at<float>(v, u) == std::numeric_limits<float>::max())
                depth_image.at<float>(v, u) = max_depth_dist;
        }
    }

    // 再做一遍插值更接近真实
    // cv::Mat resized_depth_image;
    // cv::resize(depth_image, resized_depth_image, cv::Size(48, 27));

    return depth_image;
}

pcl::PointCloud<pcl::PointXYZ> SensorSimulator::renderLidarPointcloud() {
    Eigen::Matrix3f R_wc = quat.toRotationMatrix();
    Eigen::Matrix3f R_cw = R_wc.inverse();
    pcl::PointCloud<pcl::PointXYZ> lidar_points;
    float vertical_resolution = (vertical_angle_end - vertical_angle_start) / (vertical_lines - 1);
    std::vector<pcl::PointCloud<pcl::PointXYZ>> line_clouds(vertical_lines);

    auto start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
    for (int v = 0; v < vertical_lines; ++v) {
        float vertical_angle = vertical_angle_start + v * vertical_resolution;
        float sin_vert = std::sin(vertical_angle * M_PI / 180.0);
        float cos_vert = std::cos(vertical_angle * M_PI / 180.0);

        for (int h = 0; h < horizontal_angle; ++h) {
            float horizontal_angle = h * horizontal_resolution;
            float sin_horz = std::sin(horizontal_angle * M_PI / 180.0);
            float cos_horz = std::cos(horizontal_angle * M_PI / 180.0);

            Eigen::Vector3f ray_direction(cos_vert * cos_horz, cos_vert * sin_horz, sin_vert);
            ray_direction = R_wc * ray_direction;
            Eigen::Vector3f ray_origin = pos;

            std::vector<int> pointIdxVec;
            if (octree->getIntersectedVoxelIndices(ray_origin, ray_direction, pointIdxVec, 1)) {
                pcl::PointXYZ point = cloud->points[pointIdxVec[0]];
                Eigen::Vector3f point_in_world = point.getVector3fMap();
                Eigen::Vector3f point_in_body = R_cw * (point_in_world - pos);
                if (max_lidar_dist > point_in_body.norm())
                    line_clouds[v].points.push_back(pcl::PointXYZ(point_in_body.x(), point_in_body.y(), point_in_body.z()));
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    // std::cout << "生成雷达耗时: " << elapsed.count() << " 秒" << std::endl;

    for (int i = 0; i < vertical_lines; ++i) {
        lidar_points += line_clouds[i];
    }
    lidar_points.width = lidar_points.points.size();
    lidar_points.height = 1;
    lidar_points.is_dense = true;
    return lidar_points;
}

// TODO: 不能像python那样用一个向量一次性全算吗？
void SensorSimulator::expand_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr expanded_cloud, int direction) {
    auto start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud_temp = *expanded_cloud;
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*expanded_cloud, min_point, max_point);

    float min_value = (direction == 0) ? min_point.x : min_point.y;
    float max_value = (direction == 0) ? max_point.x : max_point.y;

    // 镜像原始点云并添加到扩展点云中
    for (const auto& point : cloud_temp->points) {
        pcl::PointXYZ mirrored_point = point;
        if (direction == 0) {
            mirrored_point.x = 2 * min_value - point.x;  // 以 x 轴最小值为轴进行镜像
        } else {
            mirrored_point.y = 2 * min_value - point.y;  // 以 y 轴最小值为轴进行镜像
        }
        expanded_cloud->push_back(mirrored_point);
    }

    // 计算偏移量，保证方向上的最小值为0
    float offset = max_value - min_value;
    for (auto& point : expanded_cloud->points) {
        if (direction == 0) {
            point.x += offset;
        } else {
            point.y += offset;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    // std::cout << "点云扩张一次耗时: " << elapsed.count() << " 秒" << std::endl; // 输出耗时
}


void SensorSimulator::timerDepthCallback(const ros::TimerEvent&) {
    if (!odom_init || !render_depth)
        return;
    cv::Mat depth_iamge = renderDepthImage();
    sensor_msgs::Image ros_image;
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now();
    cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv_image.image = depth_iamge;
    cv_image.toImageMsg(ros_image);
    image_pub_.publish(ros_image);
}

void SensorSimulator::timerLidarCallback(const ros::TimerEvent&) {
    if (!odom_init || !render_lidar)
        return;
    pcl::PointCloud<pcl::PointXYZ> lidar_points = renderLidarPointcloud();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(lidar_points, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "world";

    point_cloud_pub_.publish(output);
}

void SensorSimulator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    quat.x() = msg->pose.pose.orientation.x;
    quat.y() = msg->pose.pose.orientation.y;
    quat.z() = msg->pose.pose.orientation.z;
    quat.w() = msg->pose.pose.orientation.w;

    pos.x() = msg->pose.pose.position.x;
    pos.y() = msg->pose.pose.position.y;
    pos.z() = msg->pose.pose.position.z;

    odom_init = true;
}