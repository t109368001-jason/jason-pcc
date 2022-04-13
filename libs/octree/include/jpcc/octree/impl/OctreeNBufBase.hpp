#pragma once

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNBufBase() :
    leaf_count_(0),
    branch_count_(1),
    leaf_counts_(),
    branch_counts_(),
    root_node_(new BranchNode()),
    depth_mask_(0),
    tree_dirty_flag_(false),
    octree_depth_(0),
    dynamic_depth_enabled_(false),
    bufferIndex_(0) {
  branch_counts_.fill(1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNBufBase(const OctreeNBufBase& source) :
    leaf_count_(source.leaf_count_),
    branch_count_(source.branch_count_),
    leaf_counts_(source.leaf_counts_),
    branch_counts_(source.branch_counts_),
    root_node_(new BranchNode(*source.root_node_)),
    depth_mask_(source.depth_mask_),
    max_key_(source.max_key_),
    tree_dirty_flag_(source.tree_dirty_flag_),
    octree_depth_(source.octree_depth_),
    dynamic_depth_enabled_(source.dynamic_depth_enabled_),
    bufferIndex_(source.bufferIndex_) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::~OctreeNBufBase() {
  // deallocate tree structure
  deleteTree();
  delete (root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::Iterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::begin(uindex_t maxDepth) {
  return Iterator(this, maxDepth);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::Iterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::end() {
  return Iterator();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNodeDepthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::leaf_depth_begin(uindex_t maxDepth) {
  return LeafNodeDepthFirstIterator(this, maxDepth);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNodeDepthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::leaf_depth_end() {
  return LeafNodeDepthFirstIterator();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::DepthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::depth_begin(uindex_t maxDepth) {
  return DepthFirstIterator(this, maxDepth);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::DepthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::depth_end() {
  return DepthFirstIterator();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::BreadthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::breadth_begin(uindex_t maxDepth) {
  return BreadthFirstIterator(this, maxDepth);
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::BreadthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::breadth_end() {
  return BreadthFirstIterator();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNodeBreadthIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::leaf_breadth_begin(uindex_t maxDepth) {
  return LeafNodeBreadthIterator(this, maxDepth ? maxDepth : this->octree_depth_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNodeBreadthIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::leaf_breadth_end() {
  return LeafNodeBreadthIterator(this, 0, nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>&
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::operator=(const OctreeNBufBase& source) {
  if (this == &source) { return *this; }
  deleteTree();
  leaf_count_            = source.leaf_count_;
  branch_count_          = source.branch_count_;
  leaf_counts_           = source.leaf_counts_;
  branch_counts_         = source.branch_counts_;
  root_node_             = new (BranchNode)(*(source.root_node_));
  depth_mask_            = source.depth_mask_;
  max_key_               = source.max_key_;
  bufferIndex_           = source.bufferIndex_;
  tree_dirty_flag_       = source.tree_dirty_flag_;
  octree_depth_          = source.octree_depth_;
  dynamic_depth_enabled_ = source.dynamic_depth_enabled_;
  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::setMaxVoxelIndex(uindex_t maxVoxelIndex) {
  uindex_t treeDepth;

  assert(maxVoxelIndex > 0);

  // tree depth == amount of bits of maxVoxels
  treeDepth = std::max<uindex_t>(std::min<uindex_t>(OctreeKey::maxDepth, std::ceil(std::log2(maxVoxelIndex))), 0);

  // define depthMask_ by setting a single bit to 1 at bit position == tree depth
  depth_mask_ = (1 << (treeDepth - 1));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::setTreeDepth(uindex_t depth) {
  assert(depth > 0);

  // set octree depth
  octree_depth_ = depth;

  // define depthMask_ by setting a single bit to 1 at bit position == tree depth
  depth_mask_ = (1 << (depth - 1));

  // define max. keys
  max_key_.x = max_key_.y = max_key_.z = (1 << depth) - 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::uindex_t
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getTreeDepth() const {
  return this->octree_depth_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::findLeaf(uindex_t xIndex,
                                                                                        uindex_t yIndex,
                                                                                        uindex_t zIndex) {
  // generate key
  OctreeKey key(xIndex, yIndex, zIndex);

  // check if key exist in octree
  return (findLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
const LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::findLeaf(uindex_t xIndex,
                                                                                              uindex_t yIndex,
                                                                                              uindex_t zIndex) const {
  // generate key
  OctreeKey key(xIndex, yIndex, zIndex);

  // check if key exist in octree
  return (findLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createLeaf(uindex_t xIndex,
                                                                                          uindex_t yIndex,
                                                                                          uindex_t zIndex) {
  // generate key
  OctreeKey key(xIndex, yIndex, zIndex);

  // check if key exist in octree
  return (createLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::existLeaf(uindex_t xIndex,
                                                                              uindex_t yIndex,
                                                                              uindex_t zIndex) const {
  // generate key
  OctreeKey key(xIndex, yIndex, zIndex);

  // check if key exist in octree
  return existLeaf(key);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::removeLeaf(uindex_t xIndex,
                                                                               uindex_t yIndex,
                                                                               uindex_t zIndex) {
  // generate key
  OctreeKey key(xIndex, yIndex, zIndex);

  // free voxel at key
  return (this->removeLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
std::size_t OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getLeafCount() const {
  return (leaf_count_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
std::size_t OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBranchCount() const {
  return (branch_count_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
BufferIndex OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBufferSize() const {
  return BUFFER_SIZE;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteTree() {
  if (root_node_) {
    // reset octree
    deleteBranch(*root_node_);
    leaf_count_   = 0;
    branch_count_ = 1;
    leaf_counts_.fill(0);
    branch_counts_.fill(1);

    tree_dirty_flag_ = false;
    depth_mask_      = 0;
    octree_depth_    = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::switchBuffers(const BufferIndex bufferIndex) {
  bufferIndex_ = bufferIndex;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getRootNode() {
  return (this->root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
const typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getRootNode() const {
  return (this->root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::findLeaf(const OctreeKey& key) {
  LeafContainerT* result = nullptr;
  findLeafRecursive(key, depth_mask_, root_node_, result);
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
const LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::findLeaf(
    const OctreeKey& key) const {
  LeafContainerT* result = nullptr;
  findLeafRecursive(key, depth_mask_, root_node_, result);
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createLeaf(const OctreeKey& key) {
  LeafNode*   leafNode;
  BranchNode* leafNodeParent;

  createLeafRecursive(key, depth_mask_, root_node_, leafNode, leafNodeParent, false);

  LeafContainerT* ret = leafNode->getContainerPtr();

  return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::existLeaf(const OctreeKey& key) const {
  return (findLeaf(key) != nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::removeLeaf(const OctreeKey& key) {
  if (key <= max_key_) {
    deleteLeafRecursive(key, depth_mask_, root_node_);

    // we changed the octree structure -> dirty
    tree_dirty_flag_ = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::branchHasChild(const BranchNode& branchNode,
                                                                                   ChildIndex        childIndex) const {
  // test occupancyByte for child existence
  return (branchNode.getChildPtr(bufferIndex_, childIndex) != nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBranchChildPtr(const BranchNode& branchNode,
                                                                                 ChildIndex        childIndex) {
  return branchNode.getChildPtr(bufferIndex_, childIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
const typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBranchChildPtr(const BranchNode& branchNode,
                                                                                 ChildIndex        childIndex) const {
  return branchNode.getChildPtr(bufferIndex_, childIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::setBranchChildPtr(BranchNode& branchNode,
                                                                                      ChildIndex  childIndex,
                                                                                      OctreeNode* newChild) {
  branchNode.setChildPtr(bufferIndex_, childIndex, newChild);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBranchChild(BranchNode& branchNode,
                                                                                      BufferIndex bufferIndex,
                                                                                      ChildIndex  childIndex) {
  if (branchNode.hasChild(bufferIndex, childIndex)) {
    OctreeNode* branchChild = branchNode.getChildPtr(bufferIndex, childIndex);

    switch (branchChild->getNodeType()) {
      case pcl::octree::BRANCH_NODE: {
        // free child branch recursively
        deleteBranch(*static_cast<BranchNode*>(branchChild));

        // delete unused branch
        delete (branchChild);
        for (BufferIndex _bufferIndex = 0; _bufferIndex < BUFFER_SIZE; ++_bufferIndex) {
          // set branch child pointer to 0
          branchNode.setChildPtr(_bufferIndex, childIndex, nullptr);
        }
        branch_count_--;
        branch_counts_.at(bufferIndex)--;
        break;
      }

      case pcl::octree::LEAF_NODE: {
        // push unused leaf to branch pool
        delete (branchChild);
        // set branch child pointer to 0
        branchNode.setChildPtr(bufferIndex, childIndex, nullptr);
        leaf_count_--;
        leaf_counts_.at(bufferIndex)--;
        break;
      }
      default: break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBranchChild(BranchNode& branchNode,
                                                                                      ChildIndex  childIndex) {
  deleteBranchChild(branchNode, bufferIndex_, childIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBranch(BranchNode& branchNode) {
  // delete all branch node children
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
      deleteBranchChild(branchNode, bufferIndex, childIndex);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::BranchNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createBranchChild(BranchNode& branchNode,
                                                                                 ChildIndex  childIndex) {
  auto* newBranchChild = new BranchNode();

  branchNode.setChildPtr(bufferIndex_, childIndex, newBranchChild);

  return newBranchChild;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createLeafChild(BranchNode& branchNode,
                                                                               ChildIndex  childIndex) {
  auto* newLeafChild = new LeafNode();

  branchNode.setChildPtr(bufferIndex_, childIndex, newLeafChild);

  return newLeafChild;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::uindex_t
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createLeafRecursive(const OctreeKey& key,
                                                                                   uindex_t         depthMask,
                                                                                   BranchNode*      branchNode,
                                                                                   LeafNode*&       returnLeaf,
                                                                                   BranchNode*&     parentOfLeaf,
                                                                                   bool             branchReset) {
  // branch reset -> this branch has been taken from previous buffer
  if (branchReset) {
    // we can safely remove children references
    for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
      branchNode->setChildPtr(bufferIndex_, childIndex, nullptr);
    }
  }

  // find branch child from key
  ChildIndex childIndex = key.getChildIdxWithDepthMask(depthMask);

  if (depthMask > 1) {
    // we have not reached maximum tree depth
    BranchNode* childBranch;
    bool        doNodeReset;

    doNodeReset = false;

    // if required branch does not exist
    if (!branchNode->hasChild(bufferIndex_, childIndex)) {
      // check if we find a branch node reference in previous buffer
      for (BufferIndex b = 0; b < BUFFER_SIZE; ++b) {
        if (branchNode->hasChild(b, childIndex)) {
          OctreeNode* childNode = branchNode->getChildPtr(b, childIndex);

          if (childNode->getNodeType() == pcl::octree::BRANCH_NODE) {
            childBranch = static_cast<BranchNode*>(childNode);
            branchNode->setChildPtr(bufferIndex_, childIndex, childNode);
          } else {
            // depth has changed.. child in preceding buffer is a leaf node.
            deleteBranchChild(*branchNode, b, childIndex);
            childBranch = createBranchChild(*branchNode, childIndex);
          }

          // take child branch from previous buffer
          doNodeReset = true;  // reset the branch pointer array of stolen child node
          break;
        }
      }
      if (!doNodeReset) {
        // if required branch does not exist -> create it
        childBranch = createBranchChild(*branchNode, childIndex);
      }

      branch_count_++;
      branch_counts_.at(bufferIndex_)++;
    }
    // required branch node already exists - use it
    else
      childBranch = static_cast<BranchNode*>(branchNode->getChildPtr(bufferIndex_, childIndex));

    // recursively proceed with indexed child branch
    return createLeafRecursive(key, depthMask / 2, childBranch, returnLeaf, parentOfLeaf, doNodeReset);
  }

  if (!branchNode->hasChild(bufferIndex_, childIndex)) {
    // leaf node at childIndex does not exist

    // if required leaf does not exist -> create it
    // return leaf node
    returnLeaf = createLeafChild(*branchNode, childIndex);
    leaf_count_++;
    leaf_counts_.at(bufferIndex_)++;
    parentOfLeaf = branchNode;
  } else {
    // leaf node already exist
    returnLeaf   = static_cast<LeafNode*>(branchNode->getChildPtr(bufferIndex_, childIndex));
    parentOfLeaf = branchNode;
  }

  return depthMask;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::findLeafRecursive(const OctreeKey& key,
                                                                                      uindex_t         depthMask,
                                                                                      BranchNode*      branchNode,
                                                                                      LeafContainerT*& result) const {
  // return leaf node
  ChildIndex childIndex;

  // find branch child from key
  childIndex = key.getChildIdxWithDepthMask(depthMask);

  if (depthMask > 1) {
    // we have not reached maximum tree depth
    auto* childBranch = static_cast<BranchNode*>(branchNode->getChildPtr(bufferIndex_, childIndex));

    if (childBranch)
      // recursively proceed with indexed child branch
      findLeafRecursive(key, depthMask / 2, childBranch, result);
  } else {
    // we reached leaf node level
    if (branchNode->hasChild(bufferIndex_, childIndex)) {
      // return existing leaf node
      auto* childLeaf = static_cast<LeafNode*>(branchNode->getChildPtr(bufferIndex_, childIndex));
      result          = childLeaf->getContainerPtr();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteLeafRecursive(const OctreeKey& key,
                                                                                        uindex_t         depthMask,
                                                                                        BranchNode*      branchNode) {
  // index to branch child
  ChildIndex childIndex;
  // indicates if branch is empty and can be safely removed
  bool bNoChilds;

  // find branch child from key
  childIndex = key.getChildIdxWithDepthMask(depthMask);

  if (depthMask > 1) {
    // we have not reached maximum tree depth
    // next branch child on our path through the tree
    auto* childBranch = static_cast<BranchNode*>(branchNode->getChildPtr(bufferIndex_, childIndex));

    if (childBranch) {
      // recursively explore the indexed child branch
      bool bBranchOccupied = deleteLeafRecursive(key, depthMask / 2, childBranch);

      if (!bBranchOccupied) {
        // child branch does not own any sub-child nodes anymore -> delete child branch
        deleteBranchChild(*branchNode, bufferIndex_, childIndex);
      }
    }
  } else {
    // our child is a leaf node -> delete it
    deleteBranchChild(*branchNode, bufferIndex_, childIndex);
  }

  // check if current branch still owns childs
  bNoChilds = false;
  for (childIndex = 0; childIndex < 8; childIndex++) {
    bNoChilds = branchNode->hasChild(bufferIndex_, childIndex);
    if (bNoChilds) break;
  }

  // return true if current branch can be deleted
  return (bNoChilds);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::serializeTreeCallback(LeafContainerT&,
                                                                                          const OctreeKey&) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deserializeTreeCallback(LeafContainerT&,
                                                                                            const OctreeKey&) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::octreeCanResize() const {
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::printBinary(ChildPattern childPattern) const {
  std::cout << childPattern.to_string() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
ChildPattern OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getChildPattern(
    const BranchNode& branchNode) const {
  return getChildPattern(branchNode, bufferIndex_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
ChildPattern OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getChildPattern(
    const BranchNode& branchNode, BufferIndex bufferIndex) const {
  ChildPattern childPattern;

  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    childPattern.set(childIndex, branchNode.hasChild(bufferIndex, childIndex));
  }

  return childPattern;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::BufferPattern
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBufferPattern(
    const OctreeNBufBase::BranchNode& branchNode, ChildIndex childIndex) const {
  BufferPattern bufferPattern;

  // create bit pattern
  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; bufferIndex++) {
    bufferPattern.set(bufferIndex, branchNode.hasChild(bufferIndex, childIndex));
  }

  return bufferPattern;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getIndicesByFilter(const Filter1& filter,
                                                                                       Indices&       indices) const {
  getIndicesByFilterRecursive(root_node_, filter, indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getIndicesByFilterRecursive(
    const BranchNode* branchNode, const Filter1& filter, Indices& indices) const {
  // iterate over all children
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode->hasChild(bufferIndex_, childIndex)) {
      const OctreeNode* childNode = branchNode->getChildPtr(bufferIndex_, childIndex);

      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          // recursively proceed with indexed child branchNode
          getIndicesByFilterRecursive(static_cast<const BranchNode*>(childNode), filter, indices);
          break;
        }
        case pcl::octree::LEAF_NODE: {
          const auto childLeaf = static_cast<LeafNode*>(branchNode->getChildPtr(bufferIndex_, childIndex));

          if (filter(getBufferPattern(*branchNode, childIndex))) { childLeaf->getContainer().getPointIndices(indices); }
          break;
        }
        default: break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::process(const Filter3& func,
                                                                            Indices&       indices) const {
  processRecursive(root_node_, func, indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::processRecursive(const BranchNode* branchNode,
                                                                                     const Filter3&    func,
                                                                                     Indices&          indices) const {
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode->hasChild(bufferIndex_, childIndex)) {
      const OctreeNode* childNode = branchNode->getChildPtr(bufferIndex_, childIndex);

      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          // recursively proceed with indexed child branchNode
          processRecursive(static_cast<const BranchNode*>(childNode), func, indices);
          break;
        }
        case pcl::octree::LEAF_NODE: {
          BufferIndices bufferIndices;

          for (BufferIndex b = 0; b < BUFFER_SIZE; b++) {
            if (branchNode->hasChild(b, childIndex)) {
              const auto childLeaf = static_cast<LeafNode*>(branchNode->getChildPtr(b, childIndex));

              bufferIndices.at(b) = childLeaf->getContainer().getPointIndex();
            }
          }

          if (func(bufferIndex_, getBufferPattern(*branchNode, childIndex), bufferIndices)) {
            const auto childLeaf = static_cast<LeafNode*>(branchNode->getChildPtr(bufferIndex_, childIndex));
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
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBuffer() {
  return deleteBufferRecursive(*static_cast<BranchNode*>(root_node_), bufferIndex_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBuffer(BufferIndex bufferIndex) {
  return deleteBufferRecursive(*static_cast<BranchNode*>(root_node_), bufferIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <BufferIndex BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBufferRecursive(
    OctreeNBufBase::BranchNode& branchNode, BufferIndex bufferIndex) {
  // delete all branch node children
  for (ChildIndex childIndex = 0; childIndex < 8; childIndex++) {
    if (branchNode.hasChild(bufferIndex, childIndex)) {
      OctreeNode* childNode = branchNode.getChildPtr(bufferIndex, childIndex);

      switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE: {
          bool noChild = deleteBufferRecursive(*static_cast<BranchNode*>(childNode), bufferIndex);
          branchNode.setChildPtr(bufferIndex, childIndex, nullptr);

          if (noChild) {
            delete (childNode);
            if (bufferIndex == bufferIndex_) { branch_count_--; }
            branch_counts_.at(bufferIndex)--;
          }
          break;
        }
        case pcl::octree::LEAF_NODE: {
          delete (childNode);
          branchNode.setChildPtr(bufferIndex, childIndex, nullptr);
          if (bufferIndex == bufferIndex_) { leaf_count_--; }
          leaf_counts_.at(bufferIndex)--;
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
