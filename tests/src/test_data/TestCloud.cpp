#include <test_data/octree/TestCloud.h>

namespace jpcc::octree {

using namespace std;
using namespace pcl;

ChildrenPattern getTestChildrenPattern(BufferSize bufferSelector) {
  vector<ChildrenPattern> childrenPatterns = {
      bitset<8>("10101010"), bitset<8>("11001100"), bitset<8>("11100011"), bitset<8>("11110000"),
      bitset<8>("11111000"), bitset<8>("11111100"), bitset<8>("10010010"), bitset<8>("10101010"),
      bitset<8>("10101010"), bitset<8>("10101010"), bitset<8>("10101010")  //
  };
  return childrenPatterns.at(bufferSelector);
}

PointCloud<PointXYZ>::Ptr getTestCloud(BufferSize bufferSelector) {
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
  if (getTestChildrenPattern(bufferSelector)[7]) { cloud->points.emplace_back(0.5, 0.5, 0.5); }
  if (getTestChildrenPattern(bufferSelector)[6]) { cloud->points.emplace_back(0.5, 0.5, 0.0); }
  if (getTestChildrenPattern(bufferSelector)[5]) { cloud->points.emplace_back(0.5, 0.0, 0.5); }
  if (getTestChildrenPattern(bufferSelector)[4]) { cloud->points.emplace_back(0.5, 0.0, 0.0); }
  if (getTestChildrenPattern(bufferSelector)[3]) { cloud->points.emplace_back(0.0, 0.5, 0.5); }
  if (getTestChildrenPattern(bufferSelector)[2]) { cloud->points.emplace_back(0.0, 0.5, 0.0); }
  if (getTestChildrenPattern(bufferSelector)[1]) { cloud->points.emplace_back(0.0, 0.0, 0.5); }
  if (getTestChildrenPattern(bufferSelector)[0]) { cloud->points.emplace_back(0.0, 0.0, 0.0); }
  return cloud;
}

}  // namespace jpcc::octree
