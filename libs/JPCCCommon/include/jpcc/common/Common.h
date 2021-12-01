#ifndef JPCC_COMMON_COMMON_H_
#define JPCC_COMMON_COMMON_H_

#include <assert.h>
#include <memory>

#include <pcl/point_types.h>

#include <VelodyneCapture.h>

namespace jpcc {

using std::shared_ptr;

using Point = pcl::PointXYZ;

using velodyne::Laser;

}  // namespace jpcc

#endif  // JPCC_COMMON_COMMON_H_