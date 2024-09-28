# 将点云沿XY镜像MN次，保存扩充点云

import open3d as o3d
import numpy as np

def mirror_point_cloud(pcd, N, M):
    # 获取点云的点坐标
    points = np.asarray(pcd.points)

    # 沿y方向复制并镜像N次
    for i in range(1, N + 1):
        y_max = points[:, 1].max()
        mirrored = points.copy()
        mirrored[:, 1] = y_max + (y_max - mirrored[:, 1])
        points = np.vstack((mirrored, points))

    # 沿x方向复制并镜像M次
    for j in range(1, M + 1):
        x_max = points[:, 0].max()
        mirrored = points.copy()
        mirrored[:, 0] = x_max + (x_max - mirrored[:, 0])
        points = np.vstack((mirrored, points))

    # 将复制后的点赋值回点云
    final_pcd = o3d.geometry.PointCloud()
    final_pcd.points = o3d.utility.Vector3dVector(points)
    
    return final_pcd

# 读取点云文件
pcd = o3d.io.read_point_cloud("../realworld_dense.ply")

# 设置复制次数
N = 3  # y方向复制次数
M = 1  # x方向复制次数

# 进行复制和镜像对称操作
result_pcd = mirror_point_cloud(pcd, N, M)

# 保存新的点云
o3d.io.write_point_cloud("expand.ply", result_pcd)
o3d.visualization.draw_geometries([result_pcd])