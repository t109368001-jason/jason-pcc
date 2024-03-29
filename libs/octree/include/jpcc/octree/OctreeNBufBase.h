/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */

#pragma once

#include <array>

#include <pcl/pcl_macros.h>
#include <pcl/pcl_base.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_key.h>
#include <pcl/octree/octree_nodes.h>

#include <jpcc/octree/OctreeNBufBranchNode.h>

namespace jpcc::octree {

template <typename, typename = void>
struct is_n_buf_octree : std::false_type {};

template <typename T>
struct is_n_buf_octree<T, std::void_t<decltype(&T::switchBuffers)>>
    : std::is_same<void, decltype(std::declval<T>().switchBuffers(std::declval<BufferIndex>()))> {};

template <class T>
inline constexpr bool is_n_buf_octree_v = is_n_buf_octree<T>::value;

template <BufferIndex BUFFER_SIZE,
          typename LeafContainerT   = pcl::octree::OctreeContainerPointIndices,
          typename BranchContainerT = pcl::octree::OctreeContainerEmpty>
class OctreeNBufBase {
 public:
  using ThisT      = OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>;
  using BranchNode = OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>;
  using LeafNode   = pcl::octree::OctreeLeafNode<LeafContainerT>;
  using OctreeNode = pcl::octree::OctreeNode;

  using BufferPattern = typename BranchNode::BufferPattern;

  using BranchContainer [[maybe_unused]] = BranchContainerT;
  using LeafContainer [[maybe_unused]]   = LeafContainerT;

  using OctreeKey = pcl::octree::OctreeKey;
  using uindex_t  = pcl::uindex_t;

  // public for research
  // protected:
  size_t leaf_count_;

  size_t branch_count_;

  std::array<size_t, BUFFER_SIZE> leaf_counts_;

  std::array<size_t, BUFFER_SIZE> branch_counts_;

  BranchNode* root_node_;

  uindex_t depth_mask_;

  OctreeKey max_key_;

  bool tree_dirty_flag_;

  uindex_t octree_depth_;

  bool dynamic_depth_enabled_;

  BufferIndex bufferIndex_;

 public:
  OctreeNBufBase();

  OctreeNBufBase(const OctreeNBufBase& source);

  virtual ~OctreeNBufBase();

  OctreeNBufBase& operator=(const OctreeNBufBase& source);

  void setMaxVoxelIndex(uindex_t maxVoxelIndex);

  void setTreeDepth(uindex_t depth);

  [[nodiscard]] uindex_t getTreeDepth() const;

  [[nodiscard]] LeafContainerT* createLeaf(uindex_t xIndex, uindex_t yIndex, uindex_t zIndex);

  [[nodiscard]] LeafContainerT* findLeaf(uindex_t xIndex, uindex_t yIndex, uindex_t zIndex);

  [[nodiscard]] const LeafContainerT* findLeaf(uindex_t xIndex, uindex_t yIndex, uindex_t zIndex) const;

  [[nodiscard]] bool existLeaf(uindex_t xIndex, uindex_t yIndex, uindex_t zIndex) const;

  void removeLeaf(uindex_t xIndex, uindex_t yIndex, uindex_t zIndex);

  [[nodiscard]] pcl::octree::OctreeNode* getRootNode();

  [[nodiscard]] const pcl::octree::OctreeNode* getRootNode() const;

  [[nodiscard]] size_t getLeafCount() const;

  [[nodiscard]] size_t getBranchCount() const;

  [[nodiscard]] size_t getLeafCount(BufferIndex bufferIndex) const;

  [[nodiscard]] size_t getBranchCount(BufferIndex bufferIndex) const;

  [[nodiscard]] bool branchHasChild(const BranchNode& branchNode, ChildIndex childIndex) const;

  [[nodiscard]] BufferIndex getBufferIndex() const;

  [[nodiscard]] constexpr BufferIndex getBufferSize() const;

  void deleteTree();

  void switchBuffers(BufferIndex bufferIndex);

 protected:
  [[nodiscard]] LeafContainerT* findLeaf(const OctreeKey& key);

  [[nodiscard]] const LeafContainerT* findLeaf(const OctreeKey& key) const;

  LeafContainerT* createLeaf(const OctreeKey& key);

  [[nodiscard]] bool existLeaf(const OctreeKey& key) const;

  void removeLeaf(const OctreeKey& key);

  [[nodiscard]] pcl::octree::OctreeNode* getBranchChildPtr(const BranchNode& branchNode, ChildIndex childIndex);

  [[nodiscard]] const pcl::octree::OctreeNode* getBranchChildPtr(const BranchNode& branchNode,
                                                                 ChildIndex        childIndex) const;

  void setBranchChildPtr(BranchNode& branchNode, ChildIndex childIndex, pcl::octree::OctreeNode* newChild);

  void deleteBranchChild(BranchNode& branchNode, BufferIndex bufferIndex, ChildIndex childIndex);

  void deleteBranchChild(BranchNode& branchNode, ChildIndex childIndex);

  void deleteBranch(BranchNode& branchNode);

  BranchNode* createBranchChild(BranchNode& branchNode, ChildIndex childIndex);

  LeafNode* createLeafChild(BranchNode& branchNode, ChildIndex childIndex);

  uindex_t createLeafRecursive(const OctreeKey& key,
                               uindex_t         depthMask,
                               BranchNode*      branchNode,
                               LeafNode*&       returnLeaf,
                               BranchNode*&     parentOfLeaf,
                               bool             branchReset = false);

  void findLeafRecursive(const OctreeKey& key,
                         uindex_t         depthMask,
                         BranchNode*      branchNode,
                         LeafContainerT*& result) const;

  bool deleteLeafRecursive(const OctreeKey& key, uindex_t depthMask, BranchNode* branchNode);
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeNBufBase.hpp>
