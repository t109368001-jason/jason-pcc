#include <jpcc/process/Process.h>

#include <execution>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>

#include <jpcc/octree/OctreeNBuf.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
void split(const FramePtr& input, const IndicesPtr& indices, const FramePtr& output, const FramePtr& outputNegative) {
  pcl::ExtractIndices<PointXYZINormal> extractIndices;
  extractIndices.setInputCloud(input);
  extractIndices.setIndices(indices);

  FramePtr outputTemp;
  if (output) {
    extractIndices.setNegative(false);
    if (input.get() == output.get()) {
      outputTemp = make_shared<Frame>();
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
void quantize(const FramePtr& frame, const double resolution) {
  using OctreeT = pcl::octree::OctreeBase<pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty>;
  using OctreePointCloudT = jpcc::octree::JPCCOctreePointCloud<PointXYZINormal, pcl::octree::OctreeContainerPointIndex,
                                                               pcl::octree::OctreeContainerEmpty, OctreeT>;

  OctreePointCloudT octreePointCloud(resolution);
  octreePointCloud.setFrame(frame);

  auto indices = make_shared<Indices>();
  for (auto it = octreePointCloud.begin(), it_a_end = octreePointCloud.end(); it != it_a_end; ++it) {
    const pcl::octree::OctreeNode* node = it.getCurrentOctreeNode();
    if (node->getNodeType() == pcl::octree::LEAF_NODE) {
      indices->push_back(
          dynamic_cast<const typename OctreePointCloudT::LeafNode*>(node)->getContainer().getPointIndex());
    }
  }
  process::split(frame, indices, frame, nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void quantize(const GroupOfFrame& frames, const double resolution, const bool parallel) {
  if (parallel) {
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(),
                  [&resolution](const auto& frame) { quantize(frame, resolution); });
  } else {
    std::for_each(std::execution::seq, frames.begin(), frames.end(),
                  [&resolution](const auto& frame) { quantize(frame, resolution); });
  }
}

}  // namespace jpcc::process
