#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <VelodyneCapture.h>

int main(int argc, char* argv[]) {
  std::cout << "JPCC Test App Start" << std::endl;
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.points.push_back(pcl::PointXYZ(1.0, 1.0, 1.0));
    std::cout << "cloud.size=" << cloud.size() << std::endl;
  }
  {
    std::string pcapPath;
    pcl::console::parse_argument(argc, argv, "--pcapPath", pcapPath);
    std::cout << "pcapPath=" << pcapPath << std::endl;

    if (std::filesystem::exists(pcapPath)) {
      velodyne::VLP16Capture capture(pcapPath);
      int                    frameIndex = 0;
      while (capture.isRun()) {
        std::vector<velodyne::Laser> lasers;
        if (capture.getQueueSize() > 0) {
          capture.retrieve(lasers);
          std::cout << frameIndex++ << " lasers.size()=" << lasers.size() << std::endl;
        }
      }
    }
  }
  std::cout << "JPCC Test App End" << std::endl;
  return 0;
}