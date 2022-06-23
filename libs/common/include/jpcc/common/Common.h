#pragma once

#undef NDEBUG
#include <cassert>
#include <memory>

#include <boost/throw_exception.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#define JPCC_NOT_USED(x)

namespace jpcc {

using pcl::make_shared;
using pcl::shared_ptr;

using index_t    = pcl::index_t;
using uindex_t   = pcl::uindex_t;
using Indices    = pcl::Indices;
using IndicesPtr = shared_ptr<Indices>;

using Point       = pcl::PointXYZ;
using PointNormal = pcl::PointNormal;

using Reflectivity = uint8_t;

template <typename PointT>
using Frame = pcl::PointCloud<PointT>;
template <typename PointT>
using FramePtr = typename Frame<PointT>::Ptr;

template <typename PointT>
using GroupOfFrame = std::vector<FramePtr<PointT>>;

template <class T>
struct dependent_false : std::false_type {};

template <class T>
inline constexpr bool dependent_false_v = dependent_false<T>::value;

constexpr double TO_DEGREE_MULTIPLIER = 180.0 / M_PI;

constexpr double TO_RADIAN_MULTIPLIER = M_PI / 180.0;

}  // namespace jpcc
