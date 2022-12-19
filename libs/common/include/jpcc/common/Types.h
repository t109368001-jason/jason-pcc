#pragma once

#include <chrono>
#include <map>
#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

namespace jpcc {

using std::make_shared;
using std::shared_ptr;

using Intensity = uint16_t;

using Index   = pcl::index_t;
using UIndex  = pcl::uindex_t;
using Indices = pcl::Indices;

template <typename PointT>
using PclFrame = pcl::PointCloud<PointT>;
template <typename PointT>
using PclFramePtr = typename PclFrame<PointT>::Ptr;
template <typename PointT>
using PclFrameConstPtr = typename PclFrame<PointT>::ConstPtr;

template <typename PointT>
using GroupOfPclFrame = std::vector<PclFramePtr<PointT>>;
template <typename PointT>
using GroupOfPclFrameMap = std::map<std::string, GroupOfPclFrame<PointT>>;

using Stopwatch = pcc::chrono::Stopwatch<std::chrono::steady_clock>;

}  // namespace jpcc
