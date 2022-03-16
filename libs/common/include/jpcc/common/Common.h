#pragma once

#undef NDEBUG
#include <cassert>
#include <memory>

#include <boost/throw_exception.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace jpcc {

using std::shared_ptr;

using Point  = pcl::PointXYZ;
using Normal = pcl::Normal;

struct PointNormal;

template <typename PointT = Point>
using Frame = pcl::PointCloud<PointT>;

template <typename PointT = Point>
using FramePtr = typename pcl::PointCloud<PointT>::Ptr;

}  // namespace jpcc

#include <jpcc/common/impl/Common.hpp>