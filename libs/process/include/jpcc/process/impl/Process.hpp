#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud.h>

#include <jpcc/octree/OctreeNBuf.h>

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
  using OctreeNBufT = octree::OctreeNBuf<1, pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty>;
  using OctreePointCloudT = pcl::octree::OctreePointCloud<PointT, pcl::octree::OctreeContainerPointIndex,
                                                          pcl::octree::OctreeContainerEmpty, OctreeNBufT>;

  OctreePointCloudT octree(resolution);
  octree.setInputCloud(frame);
  octree.addPointsFromInputCloud();

  auto indices = make_shared<Indices>();
  for (auto it = octree.begin(), it_a_end = octree.end(); it != it_a_end; ++it) {
    const pcl::octree::OctreeNode* node = it.getCurrentOctreeNode();
    if (node->getNodeType() == pcl::octree::LEAF_NODE) {
      indices->push_back(
          dynamic_cast<const typename OctreePointCloudT::LeafNode*>(node)->getContainer().getPointIndex());
    }
  }
  process::split<PointT>(frame, indices, frame, nullptr);
}

}  // namespace jpcc::process
