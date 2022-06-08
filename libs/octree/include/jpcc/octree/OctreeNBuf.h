#pragma once

#include <array>
#include <functional>

#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_nodes.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBufBase.h>

namespace jpcc::octree {

template <BufferIndex BUFFER_SIZE,
          typename LeafContainerT   = pcl::octree::OctreeContainerPointIndices,
          typename BranchContainerT = pcl::octree::OctreeContainerEmpty>
class OctreeNBuf : public virtual OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT> {
 public:
  using Base = OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>;

  using OctreeNode    = typename Base::OctreeNode;
  using LeafNode      = typename Base::LeafNode;
  using BranchNode    = typename Base::BranchNode;
  using BufferPattern = typename Base::BufferPattern;

  using BufferIndices      = std::array<int, BUFFER_SIZE>;
  using Filter1            = std::function<bool(const BufferPattern& bufferPattern)>;
  using Filter3            = std::function<bool(
      const BufferIndex bufferIndex, const BufferPattern& bufferPattern, const BufferIndices& bufferIndices)>;
  using LeafBranchCallback = std::function<void(const ChildIndex childIndex, const BranchNode& branchNode)>;

 public:
  void getIndicesByFilter(const Filter1& filter, Indices& indices) const;

  void getIndicesByFilterRecursive(const BranchNode* branchNode, const Filter1& filter, Indices& indices) const;

  void process(const Filter3& func, Indices& indices) const;

  void processRecursive(const BranchNode* branchNode, const Filter3& func, Indices& indices) const;

  void forEachLeafBranch(LeafBranchCallback& callback);

  void forEachLeafBranchRecursive(const BranchNode* branchNode, LeafBranchCallback& callback);

  bool deleteBuffer();

  bool deleteBuffer(BufferIndex bufferIndex);

  bool deleteBufferRecursive(BranchNode& branchNode, BufferIndex bufferIndex);
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeNBuf.hpp>
