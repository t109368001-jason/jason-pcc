#include <jpcc/process/Process.h>

#include <execution>

#include <jpcc/octree/OctreeNBuf.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
void split(const FramePtr& input, const Indices& indices, const FramePtr& output, const FramePtr& outputNegative) {
  auto indexIter = indices.begin();
  auto _frame    = jpcc::make_shared<Frame>();

  FramePtr _input;
  if (input == output || input == outputNegative) {
    _input = make_shared<Frame>(*input);
  } else {
    _input = input;
  }

  _input->subset(*output, indices);

  if (outputNegative) {
    Indices removedIndices;
    for (Index i = 0; i < _input->getPointCount() && indexIter != indices.end(); i++) {
      if (i == *indexIter) {
        indexIter++;
      } else {
        removedIndices.push_back(i);
      }
    }
    _input->subset(*outputNegative, removedIndices);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void quantize(const FramePtr& frame, const double resolution) {
  using OctreeT = pcl::octree::OctreeBase<pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty>;
  using OctreePointCloudT = jpcc::octree::JPCCOctreePointCloud<pcl::PointXYZ, pcl::octree::OctreeContainerPointIndex,
                                                               pcl::octree::OctreeContainerEmpty, OctreeT>;

  PclFramePtr<pcl::PointXYZ> pclFrame = frame->toPcl<pcl::PointXYZ>();

  OctreePointCloudT octreePointCloud(resolution);
  octreePointCloud.setFrame(pclFrame);

  Indices indices;
  for (auto it = octreePointCloud.begin(), it_a_end = octreePointCloud.end(); it != it_a_end; ++it) {
    const pcl::octree::OctreeNode* node = it.getCurrentOctreeNode();
    if (node->getNodeType() == pcl::octree::LEAF_NODE) {
      indices.push_back(
          dynamic_cast<const typename OctreePointCloudT::LeafNode*>(node)->getContainer().getPointIndex());
    }
  }
  process::split(frame, indices, frame, nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void quantize(const GroupOfFrame& frames, const double resolution, const bool parallel) {
  if (!parallel) {
    for (const auto& frame : frames) { quantize(frame, resolution); }
  } else {
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(),
                  [&resolution](const auto& frame) { quantize(frame, resolution); });
  }
}

}  // namespace jpcc::process
