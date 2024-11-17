#ifndef MAPS_HPP
#define MAPS_HPP
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>
#include <Eigen/Core>
#include "perlinnoise.hpp"

namespace mocka {

class Maps {
public:
  typedef struct BasicInfo {
    int sizeX;
    int sizeY;
    int sizeZ;
    int seed;
    double scale;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  } BasicInfo;

  BasicInfo getInfo() const;
  void setInfo(const BasicInfo &value);
  void setParam(const YAML::Node& config);
  Maps() {}
  void generate(int type);

private:
  BasicInfo info;
  // perlin3D
  double complexity;
  double fill;
  int    fractal;
  double attenuation;
  // randomMap
  double _w_l, _w_h;
  int    _ObsNum;
  // maze2D
  double width;
  int    addWallX;
  int    addWallY;
  // tree
  std::string tree_file;
  double tree_dist;
  // room
  int room_number;
  int max_windows;
  int add_ceiling;
  double window_size_min, window_size_max;

  std::uniform_real_distribution<double> dis_window_x, dis_window_z, dis_window_size;
  std::default_random_engine window_eng;

  void perlin3D();
  void maze2D();
  void randomMapGenerate();
  void Maze3DGen();
  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
  void recursizeDivisionMaze(Eigen::MatrixXi &maze);
  void optimizeMap();

  void forest();
  void generatePoissonPoints(float map_width, float map_height, float dist, std::vector<Eigen::Vector2f> &positions);
  void scaleAndTranslateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float scale_factor, Eigen::Vector2f position, Eigen::Matrix3f &rotation);
  pcl::PointCloud<pcl::PointXYZ>::Ptr generateGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &forest_cloud, float grid_size, float hight = 0.0);

  void room();
  void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud,
                           const Eigen::Matrix3f &rotation, const Eigen::Vector3f &translation);
  void generateWallWithWindows(pcl::PointCloud<pcl::PointXYZ>::Ptr wall, float L, float W, float H, int num_windows);
};

class MazePoint {
private:
  pcl::PointXYZ point;
  double dist1;
  double dist2;
  int point1;
  int point2;
  bool isdoor;

public:
  pcl::PointXYZ getPoint();
  int getPoint1();
  int getPoint2();
  double getDist1();
  double getDist2();
  void setPoint(pcl::PointXYZ p);
  void setPoint1(int p);
  void setPoint2(int p);
  void setDist1(double set);
  void setDist2(double set);
};

} // namespace mocka

#endif // MAPS_HPP
