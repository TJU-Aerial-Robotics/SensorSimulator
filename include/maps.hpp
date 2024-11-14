#ifndef MAPS_HPP
#define MAPS_HPP
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

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

  void perlin3D();
  void maze2D();
  void randomMapGenerate();
  void Maze3DGen();
  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
  void recursizeDivisionMaze(Eigen::MatrixXi &maze);
  void optimizeMap();
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
