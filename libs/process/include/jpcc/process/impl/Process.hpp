#pragma once

#include <algorithm>
#include <execution>

#include <pcl/filters/extract_indices.h>

#include <jpcc/octree/OctreeNBuf.h>

#include <jpcc/octree/OctreePointCloud.h>

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

  FramePtr<PointT> outputTemp;
  if (output) {
    extractIndices.setNegative(false);
    if (input.get() == output.get()) {
      outputTemp = make_shared<Frame<PointT>>();
      extractIndices.filter(*outputTemp);
      outputTemp->header              = input->header;
      outputTemp->sensor_origin_      = input->sensor_origin_;
      outputTemp->sensor_orientation_ = input->sensor_orientation_;
    } else {
      extractIndices.filter(*output);
    }
  }
  if (outputNegative) {
    extractIndices.setNegative(true);
    extractIndices.filter(*outputNegative);
  }
  if (outputTemp) { pcl::copyPointCloud(*outputTemp, *output); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void quantize(const FramePtr<PointT>& frame, const double resolution) {
  using OctreeT = pcl::octree::OctreeBase<pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty>;
  using OctreePointCloudT = jpcc::octree::OctreePointCloud<PointT, pcl::octree::OctreeContainerPointIndex,
                                                           pcl::octree::OctreeContainerEmpty, OctreeT>;

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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void quantize(const GroupOfFrame<PointT>& frames, const double resolution, const bool parallel) {
  if (parallel) {
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(),
                  [&resolution](const auto& frame) { quantize<PointT>(frame, resolution); });
  } else {
    std::for_each(std::execution::seq, frames.begin(), frames.end(),
                  [&resolution](const auto& frame) { quantize<PointT>(frame, resolution); });
  }
}

}  // namespace jpcc::process
