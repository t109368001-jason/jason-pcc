#include <test_data/octree/TestCloud.h>

namespace jpcc::octree {

using namespace std;
using namespace pcl;

ChildPattern getTestChildPattern(BufferIndex bufferSelector) {
  vector<ChildPattern> childPatterns = {
      ChildPattern("10101010"), ChildPattern("11001100"), ChildPattern("11100011"), ChildPattern("11110000"),
      ChildPattern("11111000"), ChildPattern("11111100"), ChildPattern("10010010"), ChildPattern("10101010"),
      ChildPattern("10101010"), ChildPattern("10101010"), ChildPattern("10101010")  //
  };
  return childPatterns.at(bufferSelector);
}

PointCloud<PointXYZ>::Ptr getTestCloud(BufferIndex bufferSelector) {
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
  if (getTestChildPattern(bufferSelector).test(7)) { cloud->emplace_back(0.5, 0.5, 0.5); }
  if (getTestChildPattern(bufferSelector).test(6)) { cloud->emplace_back(0.5, 0.5, 0.0); }
  if (getTestChildPattern(bufferSelector).test(5)) { cloud->emplace_back(0.5, 0.0, 0.5); }
  if (getTestChildPattern(bufferSelector).test(4)) { cloud->emplace_back(0.5, 0.0, 0.0); }
  if (getTestChildPattern(bufferSelector).test(3)) { cloud->emplace_back(0.0, 0.5, 0.5); }
  if (getTestChildPattern(bufferSelector).test(2)) { cloud->emplace_back(0.0, 0.5, 0.0); }
  if (getTestChildPattern(bufferSelector).test(1)) { cloud->emplace_back(0.0, 0.0, 0.5); }
  if (getTestChildPattern(bufferSelector).test(0)) { cloud->emplace_back(0.0, 0.0, 0.0); }
  return cloud;
}

}  // namespace jpcc::octree
