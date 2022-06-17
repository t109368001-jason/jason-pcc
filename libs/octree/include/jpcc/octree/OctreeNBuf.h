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
  using Base  = OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>;
  using ThisT = OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>;

  using OctreeNode    = typename Base::OctreeNode;
  using LeafNode      = typename Base::LeafNode;
  using BranchNode    = typename Base::BranchNode;
  using BufferPattern = typename Base::BufferPattern;

  using Iterator                   = pcl::octree::OctreeDepthFirstIterator<ThisT>;
  using LeafNodeDepthFirstIterator = pcl::octree::OctreeLeafNodeDepthFirstIterator<ThisT>;
  using DepthFirstIterator         = pcl::octree::OctreeDepthFirstIterator<ThisT>;
  using BreadthFirstIterator       = pcl::octree::OctreeBreadthFirstIterator<ThisT>;
  using LeafNodeBreadthIterator    = pcl::octree::OctreeLeafNodeBreadthFirstIterator<ThisT>;

  friend class pcl::octree::OctreeIteratorBase<ThisT>;
  friend class pcl::octree::OctreeDepthFirstIterator<ThisT>;
  friend class pcl::octree::OctreeBreadthFirstIterator<ThisT>;
  friend class pcl::octree::OctreeLeafNodeDepthFirstIterator<ThisT>;
  friend class pcl::octree::OctreeLeafNodeBreadthFirstIterator<ThisT>;

  using BufferIndices      = std::array<int, BUFFER_SIZE>;
  using Filter1            = std::function<bool(const BufferPattern& bufferPattern)>;
  using Filter3            = std::function<bool(
      const BufferIndex bufferIndex, const BufferPattern& bufferPattern, const BufferIndices& bufferIndices)>;
  using LeafBranchCallback = std::function<void(const ChildIndex childIndex, const BranchNode& branchNode)>;

 public:
  [[nodiscard]] Iterator begin(uindex_t maxDepth = 0);
  [[nodiscard]] Iterator end();

  [[nodiscard]] LeafNodeDepthFirstIterator leaf_depth_begin(uindex_t maxDepth = 0);
  [[nodiscard]] LeafNodeDepthFirstIterator leaf_depth_end();

  [[nodiscard]] DepthFirstIterator depth_begin(uindex_t maxDepth = 0);
  [[nodiscard]] DepthFirstIterator depth_end();

  [[nodiscard]] BreadthFirstIterator breadth_begin(uindex_t maxDepth = 0);
  [[nodiscard]] BreadthFirstIterator breadth_end();

  [[nodiscard]] LeafNodeBreadthIterator leaf_breadth_begin(uindex_t maxDepth = 0u);
  [[nodiscard]] LeafNodeBreadthIterator leaf_breadth_end();

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
