#pragma once

#undef NDEBUG
#include <cassert>
#include <memory>

#include <boost/throw_exception.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define JPCC_NOT_USED(x)

namespace jpcc {

using pcl::make_shared;
using pcl::shared_ptr;

using index_t    = pcl::index_t;
using Indices    = pcl::Indices;
using IndicesPtr = shared_ptr<Indices>;

using Point  = pcl::PointXYZ;
using Normal = pcl::Normal;

using PointNormal = pcl::PointNormal;

template <typename PointT = Point>
using Frame = pcl::PointCloud<PointT>;

template <typename PointT = Point>
using FramePtr = typename pcl::PointCloud<PointT>::Ptr;

}  // namespace jpcc
