#ifndef JPCC_OCTREE_IMPL_OCTREE_N_BUF_BRANCH_NODE_H_
#define JPCC_OCTREE_IMPL_OCTREE_N_BUF_BRANCH_NODE_H_

namespace jpcc::octree {

template <typename BranchContainerT>
OctreeNBufBranchNode<BranchContainerT>::OctreeNBufBranchNode(const uint8_t bufferSize) :
    OctreeNode(), bufferSize_(bufferSize), childMatrix_(new OctreeNode*[bufferSize][8]) {
  reset();
}

template <typename BranchContainerT>
OctreeNBufBranchNode<BranchContainerT>::OctreeNBufBranchNode(const OctreeNBufBranchNode& source) :
    OctreeNBufBranchNode(source.bufferSize_) {
  *this = source;
}

template <typename BranchContainerT>
OctreeNBufBranchNode<BranchContainerT>& OctreeNBufBranchNode<BranchContainerT>::operator=(
    const OctreeNBufBranchNode& src) {
  reset();
  if (bufferSize_ != src.bufferSize_) {
    delete childMatrix_;
    bufferSize_  = src.bufferSize_;
    childMatrix_ = new OctreeNode*[bufferSize_][8];
  }
  for (uint8_t b = 0; b < src.bufferSize_; ++b) {
    for (uint8_t i = 0; i < 8; ++i) {
      if (src.childMatrix_[b][i]) childMatrix_[b][i] = src.childMatrix_[b][i]->deepCopy();
    }
  }

  return (*this);
}

template <typename BranchContainerT>
OctreeNBufBranchNode<BranchContainerT>::~OctreeNBufBranchNode() {
  delete childMatrix_;
}

template <typename BranchContainerT>
OctreeNBufBranchNode<BranchContainerT>* OctreeNBufBranchNode<BranchContainerT>::deepCopy() const {
  return new OctreeNBufBranchNode(*this);
}

template <typename BranchContainerT>
typename OctreeNBufBranchNode<BranchContainerT>::OctreeNode* OctreeNBufBranchNode<BranchContainerT>::getChildPtr(
    uint8_t bufferIndex, uint8_t childIndex) const {
  assert((bufferIndex < bufferSize_) && (childIndex < 8));
  return childMatrix_[bufferIndex][childIndex];
}

template <typename BranchContainerT>
void OctreeNBufBranchNode<BranchContainerT>::setChildPtr(uint8_t bufferIndex, uint8_t childIndex, OctreeNode* node) {
  assert((bufferIndex < bufferSize_) && (childIndex < 8));
  childMatrix_[bufferIndex][childIndex] = node;
}

template <typename BranchContainerT>
bool OctreeNBufBranchNode<BranchContainerT>::hasChild(uint8_t bufferIndex, uint8_t childIndex) const {
  assert((bufferIndex < bufferSize_) && (childIndex < 8));
  return (childMatrix_[bufferIndex][childIndex] != nullptr);
}

template <typename BranchContainerT>
pcl::octree::node_type_t OctreeNBufBranchNode<BranchContainerT>::getNodeType() const {
  return pcl::octree::BRANCH_NODE;
}

template <typename BranchContainerT>
size_t OctreeNBufBranchNode<BranchContainerT>::getBufferSize() const {
  return bufferSize_;
}

template <typename BranchContainerT>
void OctreeNBufBranchNode<BranchContainerT>::reset() {
  memset(childMatrix_, 0, sizeof(OctreeNode*) * bufferSize_ * 8);
}

template <typename BranchContainerT>
const BranchContainerT* OctreeNBufBranchNode<BranchContainerT>::operator->() const {
  return &container_;
}

template <typename BranchContainerT>
BranchContainerT* OctreeNBufBranchNode<BranchContainerT>::operator->() {
  return &container_;
}

template <typename BranchContainerT>
const BranchContainerT& OctreeNBufBranchNode<BranchContainerT>::operator*() const {
  return container_;
}

template <typename BranchContainerT>
BranchContainerT& OctreeNBufBranchNode<BranchContainerT>::operator*() {
  return container_;
}

template <typename BranchContainerT>
const BranchContainerT& OctreeNBufBranchNode<BranchContainerT>::getContainer() const {
  return container_;
}

template <typename BranchContainerT>
BranchContainerT& OctreeNBufBranchNode<BranchContainerT>::getContainer() {
  return container_;
}

template <typename BranchContainerT>
const BranchContainerT* OctreeNBufBranchNode<BranchContainerT>::getContainerPtr() const {
  return &container_;
}

template <typename BranchContainerT>
BranchContainerT* OctreeNBufBranchNode<BranchContainerT>::getContainerPtr() {
  return &container_;
}

}  // namespace jpcc::octree

#endif  // JPCC_OCTREE_IMPL_OCTREE_N_BUF_BRANCH_NODE_H_