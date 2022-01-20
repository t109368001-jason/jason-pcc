#pragma once

namespace jpcc::octree {

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::OctreeNBufBranchNode() : OctreeNode(), childMatrix_() {
  reset();
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::OctreeNBufBranchNode(const OctreeNBufBranchNode& source) :
    OctreeNode(), childMatrix_() {
  *this = source;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator=(
    const OctreeNBufBranchNode& src) {
  if (this == &src) { return *this; }
  reset();

  for (BufferSize b = 0; b < BUFFER_SIZE; ++b) {
    for (uint8_t i = 0; i < 8; ++i) {
      if (src.childMatrix_[b][i]) childMatrix_[b][i] = src.childMatrix_[b][i]->deepCopy();
    }
  }

  return (*this);
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::~OctreeNBufBranchNode() = default;

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::deepCopy()
    const {
  return new OctreeNBufBranchNode(*this);
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
typename OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::OctreeNode*
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getChildPtr(BufferSize bufferIndex, uint8_t childIndex) const {
  assert((bufferIndex < BUFFER_SIZE) && (childIndex < 8));
  return childMatrix_[bufferIndex][childIndex];
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
void OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::setChildPtr(BufferSize  bufferIndex,
                                                                      uint8_t     childIndex,
                                                                      OctreeNode* node) {
  assert((bufferIndex < BUFFER_SIZE) && (childIndex < 8));
  childMatrix_[bufferIndex][childIndex] = node;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
bool OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::hasChild(BufferSize bufferIndex, uint8_t childIndex) const {
  assert((bufferIndex < BUFFER_SIZE) && (childIndex < 8));
  return (childMatrix_[bufferIndex][childIndex] != nullptr);
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
pcl::octree::node_type_t OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getNodeType() const {
  return pcl::octree::BRANCH_NODE;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
BufferSize OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getBufferSize() const {
  return BUFFER_SIZE;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
void OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::reset() {
  memset(childMatrix_, 0, sizeof(OctreeNode*) * BUFFER_SIZE * 8);
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
const BranchContainerT* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator->() const {
  return &container_;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
BranchContainerT* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator->() {
  return &container_;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
const BranchContainerT& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator*() const {
  return container_;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
BranchContainerT& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator*() {
  return container_;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
const BranchContainerT& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getContainer() const {
  return container_;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
BranchContainerT& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getContainer() {
  return container_;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
const BranchContainerT* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getContainerPtr() const {
  return &container_;
}

template <BufferSize BUFFER_SIZE, typename BranchContainerT>
BranchContainerT* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getContainerPtr() {
  return &container_;
}

}  // namespace jpcc::octree
