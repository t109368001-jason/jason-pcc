#pragma once

#include <cassert>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <type_traits>

#include <boost/throw_exception.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#define JPCC_NOT_USED(x)

#define THROW_IF_NOT(expression)                                                 \
  do {                                                                           \
    if (!(expression)) { BOOST_THROW_EXCEPTION(std::logic_error(#expression)); } \
  } while (0)

namespace jpcc {

using pcl::make_shared;
using pcl::shared_ptr;

using index_t    = pcl::index_t;
using uindex_t   = pcl::uindex_t;
using Indices    = pcl::Indices;
using IndicesPtr = shared_ptr<Indices>;

template <typename PointT>
using Frame = pcl::PointCloud<PointT>;
template <typename PointT>
using FramePtr = typename Frame<PointT>::Ptr;
template <typename PointT>
using FrameConstPtr = typename Frame<PointT>::ConstPtr;

template <typename PointT>
using GroupOfFrame = std::vector<FramePtr<PointT>>;
template <typename PointT>
using GroupOfFrameMap = std::map<std::string, GroupOfFrame<PointT>>;

using Stopwatch = pcc::chrono::Stopwatch<std::chrono::steady_clock>;

template <class T>
struct dependent_false : std::false_type {};

template <class T>
inline constexpr bool dependent_false_v = dependent_false<T>::value;

constexpr float MAX_INTENSITY  = 1.0;
constexpr float NULL_INTENSITY = -0.5;

}  // namespace jpcc
