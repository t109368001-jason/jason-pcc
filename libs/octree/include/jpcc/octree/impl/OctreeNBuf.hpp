#pragma once

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getIndicesByFilter(const Filter1& filter,
                                                                                   Indices&       indices) const {
  getIndicesByFilterRecursive(this->root_node_, filter, indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getIndicesByFilterRecursive(
    const BranchNode* branchNode, const Filter1& filter, Indices& indices) const {
  // iterate over all children
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode->hasChild(this->bufferIndex_, childIndex)) {
      const OctreeNode* childNode = branchNode->getChildPtr(this->bufferIndex_, childIndex);

      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          // recursively proceed with indexed child branchNode
          getIndicesByFilterRecursive(dynamic_cast<const BranchNode*>(childNode), filter, indices);
          break;
        }
        case pcl::octree::LEAF_NODE: {
          const auto childLeaf = dynamic_cast<LeafNode*>(branchNode->getChildPtr(this->bufferIndex_, childIndex));

          if (filter(branchNode->getBufferPattern(childIndex))) { childLeaf->getContainer().getPointIndices(indices); }
          break;
        }
        default: break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::process(const Filter3& func, Indices& indices) const {
  processRecursive(this->root_node_, func, indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::processRecursive(const BranchNode* branchNode,
                                                                                 const Filter3&    func,
                                                                                 Indices&          indices) const {
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode->hasChild(this->bufferIndex_, childIndex)) {
      const OctreeNode* childNode = branchNode->getChildPtr(this->bufferIndex_, childIndex);

      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          // recursively proceed with indexed child branchNode
          processRecursive(dynamic_cast<const BranchNode*>(childNode), func, indices);
          break;
        }
        case pcl::octree::LEAF_NODE: {
          BufferIndices bufferIndices;

          for (BufferIndex b = 0; b < BUFFER_SIZE; b++) {
            if (branchNode->hasChild(b, childIndex)) {
              const auto childLeaf = dynamic_cast<LeafNode*>(branchNode->getChildPtr(b, childIndex));

              bufferIndices.at(b) = childLeaf->getContainer().getPointIndex();
            }
          }

          if (func(this->bufferIndex_, branchNode->getBufferPattern(childIndex), bufferIndices)) {
            const auto childLeaf = dynamic_cast<LeafNode*>(branchNode->getChildPtr(this->bufferIndex_, childIndex));
            childLeaf->getContainer().getPointIndices(indices);
          }
          break;
        }
        default: break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::forEachLeafBranch(LeafBranchCallback& callback) {
  forEachLeafBranchRecursive(this->root_node_, callback);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::forEachLeafBranchRecursive(
    const OctreeNBuf::BranchNode* branchNode, LeafBranchCallback& callback) {
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
      if (branchNode->hasChild(bufferIndex, childIndex)) {
        const OctreeNode* childNode = branchNode->getChildPtr(bufferIndex, childIndex);

        switch (childNode->getNodeType()) {
          case pcl::octree::BRANCH_NODE: {
            // recursively proceed with indexed child branchNode
            forEachLeafBranchRecursive(dynamic_cast<const BranchNode*>(childNode), callback);
            break;
          }
          case pcl::octree::LEAF_NODE: {
            callback(childIndex, *branchNode);
            break;
          }
          default: break;
        }
        break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBuffer() {
  return deleteBufferRecursive(*dynamic_cast<BranchNode*>(this->root_node_), this->bufferIndex_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBuffer(BufferIndex bufferIndex) {
  return deleteBufferRecursive(*dynamic_cast<BranchNode*>(this->root_node_), bufferIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBufferRecursive(
    OctreeNBuf::BranchNode& branchNode, BufferIndex bufferIndex) {
  // delete all branch node children
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode.hasChild(bufferIndex, childIndex)) {
      OctreeNode* childNode = branchNode.getChildPtr(bufferIndex, childIndex);

      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          bool noChild = deleteBufferRecursive(*dynamic_cast<BranchNode*>(childNode), bufferIndex);
          branchNode.setChildPtr(bufferIndex, childIndex, nullptr);

          if (noChild) {
            delete (childNode);
            if (bufferIndex == this->bufferIndex_) { this->branch_count_--; }
            this->branch_counts_.at(bufferIndex)--;
          }
          break;
        }
        case pcl::octree::LEAF_NODE: {
          delete (childNode);
          branchNode.setChildPtr(bufferIndex, childIndex, nullptr);
          if (bufferIndex == this->bufferIndex_) { this->leaf_count_--; }
          this->leaf_counts_.at(bufferIndex)--;
          break;
        }
        default: break;
      }
    }
  }

  bool noChild = true;
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    for (BufferIndex _bufferIndex = 0; _bufferIndex < BUFFER_SIZE; ++_bufferIndex) {
      if (branchNode.hasChild(_bufferIndex, childIndex)) {
        noChild = false;
        break;
      }
    }
    if (!noChild) { break; }
  }
  return noChild;
}

}  // namespace jpcc::octree
