#include <test_data/octree/TestCloud.h>

namespace jpcc::octree {

using namespace std;
using namespace pcl;

ChildrenPattern getTestChildrenPattern(BufferSize bufferSelector) {
  vector<ChildrenPattern> childrenPatterns = {
      ChildrenPattern("10101010"), ChildrenPattern("11001100"), ChildrenPattern("11100011"),
      ChildrenPattern("11110000"), ChildrenPattern("11111000"), ChildrenPattern("11111100"),
      ChildrenPattern("10010010"), ChildrenPattern("10101010"), ChildrenPattern("10101010"),
      ChildrenPattern("10101010"), ChildrenPattern("10101010")  //
  };
  return childrenPatterns.at(bufferSelector);
}

PointCloud<PointXYZ>::Ptr getTestCloud(BufferSize bufferSelector) {
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
  if (getTestChildrenPattern(bufferSelector).test(7)) { cloud->emplace_back(0.5, 0.5, 0.5); }
  if (getTestChildrenPattern(bufferSelector).test(6)) { cloud->emplace_back(0.5, 0.5, 0.0); }
  if (getTestChildrenPattern(bufferSelector).test(5)) { cloud->emplace_back(0.5, 0.0, 0.5); }
  if (getTestChildrenPattern(bufferSelector).test(4)) { cloud->emplace_back(0.5, 0.0, 0.0); }
  if (getTestChildrenPattern(bufferSelector).test(3)) { cloud->emplace_back(0.0, 0.5, 0.5); }
  if (getTestChildrenPattern(bufferSelector).test(2)) { cloud->emplace_back(0.0, 0.5, 0.0); }
  if (getTestChildrenPattern(bufferSelector).test(1)) { cloud->emplace_back(0.0, 0.0, 0.5); }
  if (getTestChildrenPattern(bufferSelector).test(0)) { cloud->emplace_back(0.0, 0.0, 0.0); }
  return cloud;
}

}  // namespace jpcc::octree
