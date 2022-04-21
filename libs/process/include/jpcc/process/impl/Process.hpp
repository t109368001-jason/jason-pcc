#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/octree/OctreeNBufBase.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void split(const FramePtr<PointT>& input,
           const IndicesPtr&       indices,
           const FramePtr<PointT>& output,
           const FramePtr<PointT>& outputNegative) {
  pcl::ExtractIndices<PointT> extractIndices;
  extractIndices.setInputCloud(input);
  extractIndices.setIndices(indices);
  if (outputNegative) {
    extractIndices.setNegative(true);
    extractIndices.filter(*outputNegative);
  }
  if (output) {
    extractIndices.setNegative(false);
    extractIndices.filter(*output);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void quantize(const FramePtr<PointT>& frame, double resolution) {
  using OctreeNBufBaseT =
      octree::OctreeNBufBase<1, pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty>;
  using OctreePointCloudT = pcl::octree::OctreePointCloud<PointT, pcl::octree::OctreeContainerPointIndex,
                                                          pcl::octree::OctreeContainerEmpty, OctreeNBufBaseT>;

  OctreePointCloudT octree(resolution);
  octree.setInputCloud(frame);
  octree.addPointsFromInputCloud();

  auto indices = make_shared<Indices>();
  indices->resize(frame->size());
  for (auto it = octree.leaf_depth_begin(); it != octree.leaf_depth_end(); it++) {
    indices->push_back(it.getLeafContainer().getPointIndex());
  }
  process::split<PointT>(frame, indices, frame, nullptr);
}

}  // namespace jpcc::process
