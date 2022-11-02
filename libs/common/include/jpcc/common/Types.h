#pragma once

#include <chrono>
#include <map>
#include <memory>

#include <pcl/point_types.h>

#include <PCCChrono.h>
#include <PCCMemory.h>

#include <PCCPointSet.h>

namespace jpcc {

using std::make_shared;
using std::shared_ptr;

using Intensity = pcc::attr_t;

using Index      = pcl::index_t;
using UIndex     = pcl::uindex_t;
using Indices    = pcl::Indices;
using IndicesPtr = std::shared_ptr<Indices>;

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
