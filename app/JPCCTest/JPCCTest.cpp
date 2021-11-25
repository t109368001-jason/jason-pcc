#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main(int argc, char* argv[]) {
  std::cout << "JPCC Test App Start" << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.points.push_back(pcl::PointXYZ(1.0, 1.0, 1.0));
  std::cout << "cloud.size=" << cloud.size() << std::endl;

  std::cout << "JPCC Test App End" << std::endl;
  return 0;
}