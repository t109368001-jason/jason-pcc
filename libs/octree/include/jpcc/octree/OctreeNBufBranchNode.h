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

#include <bitset>
#include <cstdint>

#include <pcl/octree/octree_nodes.h>

namespace jpcc::octree {

using BufferIndex = uint16_t;

using ChildIndex = uint16_t;

using ChildPattern = std::bitset<8>;

template <BufferIndex BUFFER_SIZE = 2, typename BranchContainerT = pcl::octree::OctreeContainerEmpty>
class OctreeNBufBranchNode : public pcl::octree::OctreeNode {
 public:
  using BranchContainer = BranchContainerT;
  using OctreeNode      = pcl::octree::OctreeNode;
  using BufferPattern   = std::bitset<BUFFER_SIZE>;

 protected:
  BranchContainer container_;

  OctreeNode* childMatrix_[BUFFER_SIZE][8];

 public:
  OctreeNBufBranchNode();

  OctreeNBufBranchNode(const OctreeNBufBranchNode& source);

  OctreeNBufBranchNode& operator=(const OctreeNBufBranchNode& source);

  virtual ~OctreeNBufBranchNode() override;

  [[nodiscard]] OctreeNBufBranchNode* deepCopy() const override;

  [[nodiscard]] OctreeNode* getChildPtr(BufferIndex bufferIndex, ChildIndex childIndex) const;

  void setChildPtr(BufferIndex bufferIndex, ChildIndex childIndex, OctreeNode* node);

  [[nodiscard]] bool hasChild(BufferIndex bufferIndex, ChildIndex childIndex) const;

  [[nodiscard]] pcl::octree::node_type_t getNodeType() const override;

  void reset();

  [[nodiscard]] const BranchContainer* operator->() const;

  [[nodiscard]] BranchContainer* operator->();

  [[nodiscard]] const BranchContainer& operator*() const;

  [[nodiscard]] BranchContainer& operator*();

  [[nodiscard]] const BranchContainer& getContainer() const;

  [[nodiscard]] BranchContainer& getContainer();

  [[nodiscard]] const BranchContainer* getContainerPtr() const;

  [[nodiscard]] BranchContainer* getContainerPtr();

  [[nodiscard]] BufferIndex getBufferSize() const;

  [[nodiscard]] ChildPattern getChildPattern(BufferIndex bufferIndex) const;

  [[nodiscard]] BufferPattern getBufferPattern(ChildIndex childIndex) const;
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeNBufBranchNode.hpp>
