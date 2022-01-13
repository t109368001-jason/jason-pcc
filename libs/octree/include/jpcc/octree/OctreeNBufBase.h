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

#include <cstdint>

#define PCL_NO_PRECOMPILE
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_key.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/pcl_macros.h>

#include <jpcc/octree/OctreeNBufBranchNode.h>

namespace jpcc::octree {

template <uint8_t BUFFER_SIZE       = 2,
          typename LeafContainerT   = pcl::octree::OctreeContainerPointIndices,
          typename BranchContainerT = pcl::octree::OctreeContainerEmpty>
class OctreeNBufBase {
 public:
  using OctreeT    = OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>;
  using BranchNode = OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>;
  using LeafNode   = pcl::octree::OctreeLeafNode<LeafContainerT>;
  using OctreeNode = pcl::octree::OctreeNode;

  using BranchContainer = BranchContainerT;
  using LeafContainer   = LeafContainerT;

  using OctreeKey = pcl::octree::OctreeKey;
  using uindex_t  = unsigned int;

  using BitPattern     = uint8_t;
  using BitPatternList = Eigen::Matrix<uint8_t, BUFFER_SIZE, 1>;

  // public for research
  //  protected:
  std::size_t leaf_count_;

  std::size_t branch_count_;

  BranchNode* root_node_;

  uindex_t depth_mask_;

  OctreeKey max_key_;

  unsigned char buffer_selector_;

  bool tree_dirty_flag_;

  uindex_t octree_depth_;

  bool dynamic_depth_enabled_;

 public:
  friend class pcl::octree::OctreeIteratorBase<OctreeT>;
  friend class pcl::octree::OctreeDepthFirstIterator<OctreeT>;
  friend class pcl::octree::OctreeBreadthFirstIterator<OctreeT>;
  friend class pcl::octree::OctreeLeafNodeDepthFirstIterator<OctreeT>;
  friend class pcl::octree::OctreeLeafNodeBreadthFirstIterator<OctreeT>;

  OctreeNBufBase();

  OctreeNBufBase(const OctreeNBufBase& source);

  virtual ~OctreeNBufBase();

  using Iterator      = pcl::octree::OctreeDepthFirstIterator<OctreeT>;
  using ConstIterator = const pcl::octree::OctreeDepthFirstIterator<OctreeT>;
  Iterator       begin(uindex_t max_depth_arg = 0);
  const Iterator end();

  using LeafNodeIterator      = pcl::octree::OctreeLeafNodeDepthFirstIterator<OctreeT>;
  using ConstLeafNodeIterator = const pcl::octree::OctreeLeafNodeDepthFirstIterator<OctreeT>;

  using LeafNodeDepthFirstIterator      = pcl::octree::OctreeLeafNodeDepthFirstIterator<OctreeT>;
  using ConstLeafNodeDepthFirstIterator = const pcl::octree::OctreeLeafNodeDepthFirstIterator<OctreeT>;
  LeafNodeDepthFirstIterator leaf_depth_begin(uindex_t max_depth_arg = 0);

  const LeafNodeDepthFirstIterator leaf_depth_end();

  using DepthFirstIterator      = pcl::octree::OctreeDepthFirstIterator<OctreeT>;
  using ConstDepthFirstIterator = const pcl::octree::OctreeDepthFirstIterator<OctreeT>;
  DepthFirstIterator       depth_begin(uindex_t maxDepth_arg = 0);
  const DepthFirstIterator depth_end();

  using BreadthFirstIterator      = pcl::octree::OctreeBreadthFirstIterator<OctreeT>;
  using ConstBreadthFirstIterator = const pcl::octree::OctreeBreadthFirstIterator<OctreeT>;
  BreadthFirstIterator       breadth_begin(uindex_t max_depth_arg = 0);
  const BreadthFirstIterator breadth_end();

  using LeafNodeBreadthIterator      = pcl::octree::OctreeLeafNodeBreadthFirstIterator<OctreeT>;
  using ConstLeafNodeBreadthIterator = const pcl::octree::OctreeLeafNodeBreadthFirstIterator<OctreeT>;

  LeafNodeBreadthIterator leaf_breadth_begin(uindex_t max_depth_arg = 0u);

  const LeafNodeBreadthIterator leaf_breadth_end();

  OctreeNBufBase& operator=(const OctreeNBufBase& source);

  void setMaxVoxelIndex(uindex_t max_voxel_index_arg);

  void setTreeDepth(uindex_t depth_arg);

  uindex_t getTreeDepth() const;

  LeafContainerT* createLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg);

  LeafContainerT* findLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg);

  bool existLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg) const;

  void removeLeaf(uindex_t idx_x_arg, uindex_t idx_y_arg, uindex_t idx_z_arg);

  std::size_t getLeafCount() const;

  std::size_t getBranchCount() const;

  uint8_t getBufferSize() const;

  void deleteTree();

  void switchBuffers(const uint8_t bufferSelector);

  // public for research
  //  protected:
  pcl::octree::OctreeNode* getRootNode() const;

  LeafContainerT* findLeaf(const OctreeKey& key_arg) const;

  LeafContainerT* createLeaf(const OctreeKey& key_arg);

  bool existLeaf(const OctreeKey& key_arg) const;

  void removeLeaf(const OctreeKey& key_arg);

  bool branchHasChild(const BranchNode& branch_arg, unsigned char child_idx_arg) const;

  pcl::octree::OctreeNode* getBranchChildPtr(const BranchNode& branch_arg, unsigned char child_idx_arg) const;

  void setBranchChildPtr(BranchNode& branch_arg, unsigned char child_idx_arg, pcl::octree::OctreeNode* new_child_arg);

  uint8_t getBranchBitPattern(const BranchNode& branch_arg) const;

  uint8_t getBranchBitPattern(const BranchNode& branch_arg, uint8_t bufferSelector_arg) const;

  bool hasBranchChanges(const BranchNode& branch_arg) const;

  void deleteBranchChild(BranchNode& branch_arg, unsigned char buffer_selector_arg, unsigned char child_idx_arg);

  void deleteBranchChild(BranchNode& branch_arg, unsigned char child_idx_arg);

  void deleteBranch(BranchNode& branch_arg);

  BranchNode* createBranchChild(BranchNode& branch_arg, unsigned char child_idx_arg);

  LeafNode* createLeafChild(BranchNode& branch_arg, unsigned char child_idx_arg);

  uindex_t createLeafRecursive(const OctreeKey& key_arg,
                               uindex_t         depth_mask_arg,
                               BranchNode*      branch_arg,
                               LeafNode*&       return_leaf_arg,
                               BranchNode*&     parent_of_leaf_arg,
                               bool             branch_reset_arg = false);

  void findLeafRecursive(const OctreeKey& key_arg,
                         uindex_t         depth_mask_arg,
                         BranchNode*      branch_arg,
                         LeafContainerT*& result_arg) const;

  bool deleteLeafRecursive(const OctreeKey& key_arg, uindex_t depth_mask_arg, BranchNode* branch_arg);

  virtual void serializeTreeCallback(LeafContainerT&, const OctreeKey&);

  virtual void deserializeTreeCallback(LeafContainerT&, const OctreeKey&);

  bool octreeCanResize();

  void printBinary(char data_arg);
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeNBufBase.hpp>
