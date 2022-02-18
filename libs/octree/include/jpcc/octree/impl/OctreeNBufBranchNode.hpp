#pragma once

namespace jpcc::octree {

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::OctreeNBufBranchNode() : OctreeNode(), childMatrix_() {
  reset();
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::OctreeNBufBranchNode(const OctreeNBufBranchNode& source) :
    OctreeNode(), childMatrix_() {
  *this = source;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator=(
    const OctreeNBufBranchNode& source) {
  if (this == &source) { return *this; }
  reset();

  for (BufferIndex bufferIndex = 0; bufferIndex < BUFFER_SIZE; ++bufferIndex) {
    for (ChildIndex childIndex = 0; childIndex < 8; ++childIndex) {
      if (source.childMatrix_[bufferIndex][childIndex])
        childMatrix_[bufferIndex][childIndex] = source.childMatrix_[bufferIndex][childIndex]->deepCopy();
    }
  }

  return (*this);
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::~OctreeNBufBranchNode() = default;

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::deepCopy()
    const {
  return new OctreeNBufBranchNode(*this);
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
typename OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::OctreeNode*
OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getChildPtr(BufferIndex bufferIndex, ChildIndex childIndex) const {
  assert((bufferIndex < BUFFER_SIZE) && (childIndex < 8));
  return childMatrix_[bufferIndex][childIndex];
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
void OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::setChildPtr(BufferIndex bufferIndex,
                                                                      ChildIndex  childIndex,
                                                                      OctreeNode* node) {
  assert((bufferIndex < BUFFER_SIZE) && (childIndex < 8));
  childMatrix_[bufferIndex][childIndex] = node;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
bool OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::hasChild(BufferIndex bufferIndex,
                                                                   ChildIndex  childIndex) const {
  assert((bufferIndex < BUFFER_SIZE) && (childIndex < 8));
  return (childMatrix_[bufferIndex][childIndex] != nullptr);
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
pcl::octree::node_type_t OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getNodeType() const {
  return pcl::octree::BRANCH_NODE;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
BufferIndex OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getBufferSize() const {
  return BUFFER_SIZE;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
void OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::reset() {
  memset(childMatrix_, 0, sizeof(OctreeNode*) * BUFFER_SIZE * 8);
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
const BranchContainerT* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator->() const {
  return &container_;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
BranchContainerT* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator->() {
  return &container_;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
const BranchContainerT& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator*() const {
  return container_;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
BranchContainerT& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::operator*() {
  return container_;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
const BranchContainerT& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getContainer() const {
  return container_;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
BranchContainerT& OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getContainer() {
  return container_;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
const BranchContainerT* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getContainerPtr() const {
  return &container_;
}

template <BufferIndex BUFFER_SIZE, typename BranchContainerT>
BranchContainerT* OctreeNBufBranchNode<BUFFER_SIZE, BranchContainerT>::getContainerPtr() {
  return &container_;
}

}  // namespace jpcc::octree
