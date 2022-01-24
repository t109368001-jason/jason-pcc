#pragma once

#undef NDEBUG
#include <cassert>
#include <memory>

#include <boost/throw_exception.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace jpcc {

using std::shared_ptr;

using Point = pcl::PointXYZ;

using Frame = pcl::PointCloud<Point>;

}  // namespace jpcc
