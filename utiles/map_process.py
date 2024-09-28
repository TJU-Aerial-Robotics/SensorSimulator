# 将全局地图裁剪并保存
# 先将点云旋转至水平，地面也保持水平，通过修改yaw_angle_degrees, pitch_angle_degrees
# 然后将点云地面移至0，通过修改translation_no
# 然后设定裁剪范围min_bound, max_bound
# 最后平移点云至原点，通过修改move_no

import open3d as o3d
import numpy as np

# 定义旋转角度（偏航角和俯仰角）
yaw_angle_degrees = -120  # 偏航角（以度为单位）
pitch_angle_degrees = 2.2  # 俯仰角（以度为单位）
translation_no = np.array([0, 0, 2])  # 平移2米到Z方向

# 点云腐蚀
def erode_point_cloud(pcd, min_neighbors=5, radius=0.02, z_min=0.5, z_max=2.8):
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    # 存储满足腐蚀条件的点
    points_to_keep = []
    points = np.asarray(pcd.points)
    for i in range(points.shape[0]):
        z_value = points[i][2]
        # 如果z值小于阈值，则直接保留该点
        if z_value < z_min or z_value > z_max:
            points_to_keep.append(points[i])
        else:
            # 查找该点的邻域点
            [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], radius)
            # 保留邻域点数量大于等于min_neighbors的点
            if k >= min_neighbors:
                points_to_keep.append(points[i])
    # 创建腐蚀后的点云
    eroded_pcd = o3d.geometry.PointCloud()
    eroded_pcd.points = o3d.utility.Vector3dVector(np.array(points_to_keep))
    return eroded_pcd

# 1. 加载点云数据
point_cloud = o3d.io.read_point_cloud("dense_forest.pcd")  # 替换为点云文件的路径

# 3. 将角度转换为弧度
yaw_angle_radians = np.radians(yaw_angle_degrees)
pitch_angle_radians = np.radians(pitch_angle_degrees)

yaw_rotation = np.array([[np.cos(yaw_angle_radians), -np.sin(yaw_angle_radians), 0],
                         [np.sin(yaw_angle_radians), np.cos(yaw_angle_radians), 0],
                         [0, 0, 1]])

pitch_rotation = np.array([[np.cos(pitch_angle_radians), 0, np.sin(pitch_angle_radians)],
                           [0, 1, 0],
                           [-np.sin(pitch_angle_radians), 0, np.cos(pitch_angle_radians)]])

# 5. 组合旋转矩阵 R old->new
R_on = np.dot(yaw_rotation, pitch_rotation)  # 内旋是右乘，先yaw后pitch
point_cloud.points = o3d.utility.Vector3dVector(np.dot(np.asarray(point_cloud.points), R_on) + translation_no)


# 6. 定义裁剪范围
min_bound = np.array([-115.0, 33.0, 0.1])  # 最小点坐标
max_bound = np.array([-25.0, 58.0, 20])    # 最大点坐标

# 7. 使用crop函数裁剪点云
cropped_point_cloud = point_cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound))

move_no = np.array([-115.0, 33.0, 0.5])
cropped_point_cloud.points = o3d.utility.Vector3dVector(np.asarray(cropped_point_cloud.points) - move_no)

# 8.手动生成地面点云 [90, 30, 0] 
x_min, x_max = 0, 90
y_min, y_max = 0, 25
resolution = 0.1
# 生成地面点云
x_coords = np.arange(x_min, x_max, resolution)
y_coords = np.arange(y_min, y_max, resolution)
x_grid, y_grid = np.meshgrid(x_coords, y_coords)
z_grid = np.zeros_like(x_grid) + 0.0

# 9.点云和地面组合
ground_points = np.vstack((x_grid.flatten(), y_grid.flatten(), z_grid.flatten())).T
original_points = np.asarray(cropped_point_cloud.points)
combined_points = np.vstack((original_points, ground_points))
combined_ply = o3d.geometry.PointCloud()
combined_ply.points = o3d.utility.Vector3dVector(combined_points)

# 10.构建占据栅格地图将导致树变粗，这里先腐蚀一次
eroded_pcd = erode_point_cloud(combined_ply, min_neighbors=8, radius=0.05)

o3d.io.write_point_cloud("realworld_dense.ply", eroded_pcd, write_ascii=True)
o3d.visualization.draw_geometries([eroded_pcd])