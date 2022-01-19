#pragma clang diagnostic push
#pragma ide diagnostic   ignored "OCUnusedGlobalDeclarationInspection"
#ifndef JPCC_OCTREE_IMPL_OCTREE_N_BUF_BASE_H_
#define JPCC_OCTREE_IMPL_OCTREE_N_BUF_BASE_H_

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNBufBase() :
    leaf_count_(0),
    branch_count_(1),
    root_node_(new BranchNode()),
    depth_mask_(0),
    buffer_selector_(0),
    tree_dirty_flag_(false),
    octree_depth_(0),
    dynamic_depth_enabled_(false) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNBufBase(const OctreeNBufBase& source) :
    leaf_count_(source.leaf_count_),
    branch_count_(source.branch_count_),
    root_node_(new BranchNode(*source.root_node_)),
    depth_mask_(source.depth_mask_),
    max_key_(source.max_key_),
    buffer_selector_(source.buffer_selector_),
    tree_dirty_flag_(source.tree_dirty_flag_),
    octree_depth_(source.octree_depth_),
    dynamic_depth_enabled_(source.dynamic_depth_enabled_) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::~OctreeNBufBase() {
  // deallocate tree structure
  deleteTree();
  delete (root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::Iterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::begin(uindex_t max_depth_arg) {
  return Iterator(this, max_depth_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::Iterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::end() {
  return Iterator();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNodeDepthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::leaf_depth_begin(uindex_t max_depth_arg) {
  return LeafNodeDepthFirstIterator(this, max_depth_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNodeDepthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::leaf_depth_end() {
  return LeafNodeDepthFirstIterator();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::DepthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::depth_begin(uindex_t maxDepth_arg) {
  return DepthFirstIterator(this, maxDepth_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::DepthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::depth_end() {
  return DepthFirstIterator();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::BreadthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::breadth_begin(uindex_t max_depth_arg) {
  return BreadthFirstIterator(this, max_depth_arg);
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::BreadthFirstIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::breadth_end() {
  return BreadthFirstIterator();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNodeBreadthIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::leaf_breadth_begin(uindex_t max_depth_arg) {
  return LeafNodeBreadthIterator(this, max_depth_arg ? max_depth_arg : this->octree_depth_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNodeBreadthIterator
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::leaf_breadth_end() {
  return LeafNodeBreadthIterator(this, 0, nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>&
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::operator=(const OctreeNBufBase& source) {
  if (this == &source) { return *this; }
  leaf_count_            = source.leaf_count_;
  branch_count_          = source.branch_count_;
  root_node_             = new (BranchNode)(*(source.root_node_));
  depth_mask_            = source.depth_mask_;
  max_key_               = source.max_key_;
  buffer_selector_       = source.buffer_selector_;
  tree_dirty_flag_       = source.tree_dirty_flag_;
  octree_depth_          = source.octree_depth_;
  dynamic_depth_enabled_ = source.dynamic_depth_enabled_;
  return (*this);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::setMaxVoxelIndex(uindex_t max_voxel_index_arg) {
  uindex_t treeDepth;

  assert(max_voxel_index_arg > 0);

  // tree depth == amount of bits of maxVoxels
  treeDepth = std::max<uindex_t>(std::min<uindex_t>(OctreeKey::maxDepth, std::ceil(std::log2(max_voxel_index_arg))), 0);

  // define depthMask_ by setting a single bit to 1 at bit position == tree depth
  depth_mask_ = (1 << (treeDepth - 1));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::setTreeDepth(uindex_t depth_arg) {
  assert(depth_arg > 0);

  // set octree depth
  octree_depth_ = depth_arg;

  // define depthMask_ by setting a single bit to 1 at bit position == tree depth
  depth_mask_ = (1 << (depth_arg - 1));

  // define max. keys
  max_key_.x = max_key_.y = max_key_.z = (1 << depth_arg) - 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::uindex_t
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getTreeDepth() const {
  return this->octree_depth_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::findLeaf(uindex_t idx_x_arg,
                                                                                        uindex_t idx_y_arg,
                                                                                        uindex_t idx_z_arg) {
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // check if key exist in octree
  return (findLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createLeaf(uindex_t idx_x_arg,
                                                                                          uindex_t idx_y_arg,
                                                                                          uindex_t idx_z_arg) {
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // check if key exist in octree
  return (createLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::existLeaf(uindex_t idx_x_arg,
                                                                              uindex_t idx_y_arg,
                                                                              uindex_t idx_z_arg) const {
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // check if key exist in octree
  return existLeaf(key);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::removeLeaf(uindex_t idx_x_arg,
                                                                               uindex_t idx_y_arg,
                                                                               uindex_t idx_z_arg) {
  // generate key
  OctreeKey key(idx_x_arg, idx_y_arg, idx_z_arg);

  // free voxel at key
  return (this->removeLeaf(key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
std::size_t OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getLeafCount() const {
  return (leaf_count_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
std::size_t OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBranchCount() const {
  return (branch_count_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
uint8_t OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBufferSize() const {
  return BUFFER_SIZE;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteTree() {
  if (root_node_) {
    // reset octree
    deleteBranch(*root_node_);
    leaf_count_   = 0;
    branch_count_ = 1;

    tree_dirty_flag_ = false;
    depth_mask_      = 0;
    octree_depth_    = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::switchBuffers(const uint8_t bufferSelector) {
  buffer_selector_ = bufferSelector;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getRootNode() const {
  return (this->root_node_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::findLeaf(
    const OctreeKey& key_arg) const {
  LeafContainerT* result = nullptr;
  findLeafRecursive(key_arg, depth_mask_, root_node_, result);
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
LeafContainerT* OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createLeaf(const OctreeKey& key_arg) {
  LeafNode*   leaf_node;
  BranchNode* leaf_node_parent;

  createLeafRecursive(key_arg, depth_mask_, root_node_, leaf_node, leaf_node_parent, false);

  LeafContainerT* ret = leaf_node->getContainerPtr();

  return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::existLeaf(const OctreeKey& key_arg) const {
  return (findLeaf(key_arg) != nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::removeLeaf(const OctreeKey& key_arg) {
  if (key_arg <= max_key_) {
    deleteLeafRecursive(key_arg, depth_mask_, root_node_);

    // we changed the octree structure -> dirty
    tree_dirty_flag_ = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::branchHasChild(const BranchNode& branch_arg,
                                                                                   unsigned char child_idx_arg) const {
  // test occupancyByte for child existence
  return (branch_arg.getChildPtr(buffer_selector_, child_idx_arg) != nullptr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::OctreeNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBranchChildPtr(const BranchNode& branch_arg,
                                                                                 unsigned char child_idx_arg) const {
  return branch_arg.getChildPtr(buffer_selector_, child_idx_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::setBranchChildPtr(BranchNode&   branch_arg,
                                                                                      unsigned char child_idx_arg,
                                                                                      OctreeNode*   new_child_arg) {
  branch_arg.setChildPtr(buffer_selector_, child_idx_arg, new_child_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
uint8_t OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBranchBitPattern(
    const BranchNode& branch_arg) const {
  uint8_t node_bits;

  // create bit pattern
  node_bits = 0;
  for (uint8_t i = 0; i < 8; i++) {
    const OctreeNode* child = branch_arg.getChildPtr(buffer_selector_, i);
    node_bits |= static_cast<uint8_t>((child != nullptr) << i);
  }

  return (node_bits);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
uint8_t OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBranchBitPattern(
    const BranchNode& branch_arg, uint8_t bufferSelector_arg) const {
  uint8_t node_bits;

  // create bit pattern
  node_bits = 0;
  for (uint8_t i = 0; i < 8; i++) {
    const OctreeNode* child = branch_arg.getChildPtr(bufferSelector_arg, i);
    node_bits |= static_cast<uint8_t>((child != nullptr) << i);
  }

  return (node_bits);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::BufferPattern
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::getBranchBufferPattern(
    const OctreeNBufBase::BranchNode& branch_arg, uint8_t childrenIdx) const {
  BufferPattern bufferPattern;

  // create bit pattern
  for (uint8_t b = 0; b < BUFFER_SIZE; b++) {
    const OctreeNode* child = branch_arg.getChildPtr(b, childrenIdx);
    bufferPattern[b]        = (child != nullptr);
  }

  return bufferPattern;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBranchChild(BranchNode&   branch_arg,
                                                                                      unsigned char buffer_selector_arg,
                                                                                      unsigned char child_idx_arg) {
  if (branch_arg.hasChild(buffer_selector_arg, child_idx_arg)) {
    OctreeNode* branchChild = branch_arg.getChildPtr(buffer_selector_arg, child_idx_arg);

    switch (branchChild->getNodeType()) {
      case pcl::octree::BRANCH_NODE: {
        // free child branch recursively
        deleteBranch(*static_cast<BranchNode*>(branchChild));

        // delete unused branch
        delete (branchChild);
        break;
      }

      case pcl::octree::LEAF_NODE: {
        // push unused leaf to branch pool
        delete (branchChild);
        break;
      }
      default: break;
    }

    // set branch child pointer to 0
    branch_arg.setChildPtr(buffer_selector_arg, child_idx_arg, nullptr);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBranchChild(BranchNode&   branch_arg,
                                                                                      unsigned char child_idx_arg) {
  deleteBranchChild(branch_arg, buffer_selector_, child_idx_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteBranch(BranchNode& branch_arg) {
  // delete all branch node children
  for (char i = 0; i < 8; i++) {
    for (uint8_t b = 0; b < BUFFER_SIZE; ++b) {
      if (branch_arg.getChildPtr(b, i)) {
        deleteBranchChild(branch_arg, b, i);
        for (uint8_t b_ = 0; b_ < BUFFER_SIZE; ++b_) { branch_arg.setChildPtr(b_, i, nullptr); }
        break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::BranchNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createBranchChild(BranchNode&   branch_arg,
                                                                                 unsigned char child_idx_arg) {
  auto* new_branch_child = new BranchNode();

  branch_arg.setChildPtr(buffer_selector_, child_idx_arg, static_cast<OctreeNode*>(new_branch_child));

  return new_branch_child;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::LeafNode*
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createLeafChild(BranchNode&   branch_arg,
                                                                               unsigned char child_idx_arg) {
  auto* new_leaf_child = new LeafNode();

  branch_arg.setChildPtr(buffer_selector_, child_idx_arg, new_leaf_child);

  return new_leaf_child;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
typename OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::uindex_t
OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::createLeafRecursive(const OctreeKey& key_arg,
                                                                                   uindex_t         depth_mask_arg,
                                                                                   BranchNode*      branch_arg,
                                                                                   LeafNode*&       return_leaf_arg,
                                                                                   BranchNode*&     parent_of_leaf_arg,
                                                                                   bool             branch_reset_arg) {
  // branch reset -> this branch has been taken from previous buffer
  if (branch_reset_arg) {
    // we can safely remove children references
    for (unsigned char child_idx = 0; child_idx < 8; child_idx++) {
      branch_arg->setChildPtr(buffer_selector_, child_idx, nullptr);
    }
  }

  // find branch child from key
  unsigned char child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  if (depth_mask_arg > 1) {
    // we have not reached maximum tree depth
    BranchNode* child_branch;
    bool        doNodeReset;

    doNodeReset = false;

    // if required branch does not exist
    if (!branch_arg->hasChild(buffer_selector_, child_idx)) {
      // check if we find a branch node reference in previous buffer
      for (uint8_t b = 0; b < BUFFER_SIZE; ++b) {
        if (b == buffer_selector_) { continue; }
        if (branch_arg->hasChild(b, child_idx)) {
          OctreeNode* child_node = branch_arg->getChildPtr(b, child_idx);

          if (child_node->getNodeType() == pcl::octree::BRANCH_NODE) {
            child_branch = static_cast<BranchNode*>(child_node);
            branch_arg->setChildPtr(buffer_selector_, child_idx, child_node);
          } else {
            // depth has changed.. child in preceding buffer is a leaf node.
            deleteBranchChild(*branch_arg, b, child_idx);
            child_branch = createBranchChild(*branch_arg, child_idx);
          }

          // take child branch from previous buffer
          doNodeReset = true;  // reset the branch pointer array of stolen child node
        }
      }
      if (!doNodeReset) {
        // if required branch does not exist -> create it
        child_branch = createBranchChild(*branch_arg, child_idx);
      }

      branch_count_++;
    }
    // required branch node already exists - use it
    else
      child_branch = static_cast<BranchNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));

    // recursively proceed with indexed child branch
    return createLeafRecursive(key_arg, depth_mask_arg / 2, child_branch, return_leaf_arg, parent_of_leaf_arg,
                               doNodeReset);
  }

  bool stolenChildNode = false;
  // branch childs are leaf nodes
  LeafNode* child_leaf;
  if (!branch_arg->hasChild(buffer_selector_, child_idx)) {
    // leaf node at child_idx does not exist

    for (uint8_t b = 0; b < BUFFER_SIZE; ++b) {
      if (b == buffer_selector_) { continue; }
      // check if we can take copy a reference from previous buffer
      if (branch_arg->hasChild(b, child_idx)) {
        OctreeNode* child_node = branch_arg->getChildPtr(b, child_idx);
        if (child_node->getNodeType() == pcl::octree::LEAF_NODE) {
          child_leaf                 = static_cast<LeafNode*>(child_node);
          child_leaf->getContainer() = LeafContainer();  // Clear contents of leaf
          branch_arg->setChildPtr(buffer_selector_, child_idx, child_node);
        } else {
          // depth has changed.. child in preceding buffer is a leaf node.
          deleteBranchChild(*branch_arg, b, child_idx);
          child_leaf = createLeafChild(*branch_arg, child_idx);
        }
        leaf_count_++;
        stolenChildNode = true;
      }
    }
    if (!stolenChildNode) {
      // if required leaf does not exist -> create it
      child_leaf = createLeafChild(*branch_arg, child_idx);
      leaf_count_++;
    }

    // return leaf node
    return_leaf_arg    = child_leaf;
    parent_of_leaf_arg = branch_arg;
  } else {
    // leaf node already exist
    return_leaf_arg    = static_cast<LeafNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));
    parent_of_leaf_arg = branch_arg;
  }

  return depth_mask_arg;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::findLeafRecursive(
    const OctreeKey& key_arg, uindex_t depth_mask_arg, BranchNode* branch_arg, LeafContainerT*& result_arg) const {
  // return leaf node
  unsigned char child_idx;

  // find branch child from key
  child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  if (depth_mask_arg > 1) {
    // we have not reached maximum tree depth
    BranchNode* child_branch;
    child_branch = static_cast<BranchNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));

    if (child_branch)
      // recursively proceed with indexed child branch
      findLeafRecursive(key_arg, depth_mask_arg / 2, child_branch, result_arg);
  } else {
    // we reached leaf node level
    if (branch_arg->hasChild(buffer_selector_, child_idx)) {
      // return existing leaf node
      auto* leaf_node = static_cast<LeafNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));
      result_arg      = leaf_node->getContainerPtr();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deleteLeafRecursive(const OctreeKey& key_arg,
                                                                                        uindex_t         depth_mask_arg,
                                                                                        BranchNode*      branch_arg) {
  // index to branch child
  unsigned char child_idx;
  // indicates if branch is empty and can be safely removed
  bool bNoChilds;

  // find branch child from key
  child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

  if (depth_mask_arg > 1) {
    // we have not reached maximum tree depth
    BranchNode* child_branch;

    // next branch child on our path through the tree
    child_branch = static_cast<BranchNode*>(branch_arg->getChildPtr(buffer_selector_, child_idx));

    if (child_branch) {
      // recursively explore the indexed child branch
      bool bBranchOccupied = deleteLeafRecursive(key_arg, depth_mask_arg / 2, child_branch);

      if (!bBranchOccupied) {
        // child branch does not own any sub-child nodes anymore -> delete child branch
        deleteBranchChild(*branch_arg, buffer_selector_, child_idx);
        branch_count_--;
      }
    }
  } else {
    // our child is a leaf node -> delete it
    deleteBranchChild(*branch_arg, buffer_selector_, child_idx);
    leaf_count_--;
  }

  // check if current branch still owns childs
  bNoChilds = false;
  for (child_idx = 0; child_idx < 8; child_idx++) {
    bNoChilds = branch_arg->hasChild(buffer_selector_, child_idx);
    if (bNoChilds) break;
  }

  // return true if current branch can be deleted
  return (bNoChilds);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::serializeTreeCallback(LeafContainerT&,
                                                                                          const OctreeKey&) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::deserializeTreeCallback(LeafContainerT&,
                                                                                            const OctreeKey&) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
bool OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::octreeCanResize() {
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <uint8_t BUFFER_SIZE, typename LeafContainerT, typename BranchContainerT>
void OctreeNBufBase<BUFFER_SIZE, LeafContainerT, BranchContainerT>::printBinary(char data_arg) {
  unsigned char mask = 1;  // Bit mask

  // Extract the bits
  for (int i = 0; i < 8; i++) {
    // Mask each bit in the byte and print it
    std::cout << ((data_arg & (mask << i)) ? "1" : "0");
  }
  std::cout << std::endl;
}

}  // namespace jpcc::octree

#endif  // JPCC_OCTREE_IMPL_OCTREE_N_BUF_BASE_H_
#pragma clang diagnostic pop
