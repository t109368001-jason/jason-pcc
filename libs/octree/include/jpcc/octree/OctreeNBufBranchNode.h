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

#include <pcl/octree/octree_nodes.h>

namespace jpcc::octree {

using BufferSize = uint16_t;

using ChildrenIndex = uint16_t;

template <BufferSize BUFFER_SIZE = 2, typename BranchContainerT = pcl::octree::OctreeContainerEmpty>
class OctreeNBufBranchNode : public pcl::octree::OctreeNode {
 public:
  using BranchContainer = BranchContainerT;
  using OctreeNode      = pcl::octree::OctreeNode;

 protected:
  BranchContainer container_;

  OctreeNode* childMatrix_[BUFFER_SIZE][8];

 public:
  OctreeNBufBranchNode();

  OctreeNBufBranchNode(const OctreeNBufBranchNode& source);

  OctreeNBufBranchNode& operator=(const OctreeNBufBranchNode& src);

  ~OctreeNBufBranchNode() override;

  OctreeNBufBranchNode* deepCopy() const override;

  [[nodiscard]] OctreeNode* getChildPtr(BufferSize bufferIndex, ChildrenIndex childIndex) const;

  void setChildPtr(BufferSize bufferIndex, ChildrenIndex childIndex, OctreeNode* node);

  [[nodiscard]] bool hasChild(BufferSize bufferIndex, ChildrenIndex childIndex) const;

  [[nodiscard]] pcl::octree::node_type_t getNodeType() const override;

  [[nodiscard]] BufferSize getBufferSize() const;

  void reset();

  const BranchContainer* operator->() const;

  BranchContainer* operator->();

  const BranchContainer& operator*() const;

  BranchContainer& operator*();

  const BranchContainer& getContainer() const;

  BranchContainer& getContainer();

  const BranchContainer* getContainerPtr() const;

  BranchContainer* getContainerPtr();
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeNBufBranchNode.hpp>
