#pragma once

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::JPCCOctreePointCloud(double resolution) :
    Base(resolution) {
  this->defineBoundingBox(resolution * 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::setFrame(FrameConstPtr<PointT> frame) {
  if constexpr (is_n_buf_octree_v<OctreeT>) {
    this->deleteBuffer(this->getBufferIndex());
    this->setInputCloud(frame);
    this->addPointsFromInputCloud();
  } else {
    this->setInputCloud(frame);
    this->addPointsFromInputCloud();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::setFrame(BufferIndex bufferIndex,
                                                                                       FrameConstPtr<PointT> frame) {
  if constexpr (is_n_buf_octree_v<OctreeT>) {
    this->switchBuffers(bufferIndex);
    setFrame(frame);
  } else {
    static_assert(dependent_false_v<OctreeT>, "setFrame only support for OctreeNBuf, please use setFrame");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::addFrame(FrameConstPtr<PointT> frame) {
  if constexpr (is_n_buf_octree_v<OctreeT>) {
    this->setInputCloud(frame);
    this->addPointsFromInputCloud();
  } else {
    this->setInputCloud(frame);
    this->addPointsFromInputCloud();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::addFrame(BufferIndex bufferIndex,
                                                                                       FrameConstPtr<PointT> frame) {
  if constexpr (is_n_buf_octree_v<OctreeT>) {
    this->switchBuffers(bufferIndex);
    addFrame(frame);
  } else {
    static_assert(dependent_false_v<OctreeT>,
                  "addFrame(BufferIndex, FramePtr) only support for OctreeNBuf, please use addFrame(FramePtr)");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::deletePointFromCloud(
    const PointT& toDeletePoint, PointCloudPtr cloud) {
  if constexpr (std::is_same_v<LeafContainerT, OctreeContainerEditableIndex>) {
    OctreeKey toDeleteKey;
    this->genOctreeKeyforPoint(toDeletePoint, toDeleteKey);

    LeafContainerT* toDeleteContainer = this->findLeaf(toDeleteKey);
    assert(toDeleteContainer != nullptr);
    LeafContainerT* lastContainer = this->findLeafAtPoint(cloud->back());
    assert(lastContainer != nullptr);

    PointT& toDeleteAndReplacePoint = (*cloud)[toDeleteContainer->getPointIndex()];
    assert(toDeleteAndReplacePoint.x == toDeletePoint.x);
    assert(toDeleteAndReplacePoint.y == toDeletePoint.y);
    assert(toDeleteAndReplacePoint.z == toDeletePoint.z);
    PointT& toBackupAndDeletePoint = (*cloud)[lastContainer->getPointIndex()];
    toDeleteAndReplacePoint        = toBackupAndDeletePoint;
    lastContainer->setPointIndex(toDeleteContainer->getPointIndex());

    cloud->points.pop_back();
    cloud->width  = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;

    // remove container
    this->removeLeaf(toDeleteKey);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void jpcc::octree::JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::addPointIdx(
    uindex_t point_idx_arg) {
  OctreeKey key;

  assert(point_idx_arg < this->input_->size());

  const PointT& point = (*this->input_)[point_idx_arg];

  // make sure bounding box is big enough
  this->adoptBoundingBoxToPoint(point);

  // generate key
  this->genOctreeKeyforPoint(point, key);

  LeafNode*   leaf_node;
  BranchNode* parent_branch_of_leaf_node;
  auto        depth_mask =
      this->createLeafRecursive(key, this->depth_mask_, this->root_node_, leaf_node, parent_branch_of_leaf_node);

  if (this->dynamic_depth_enabled_ && depth_mask) {
    // get amount of objects in leaf container
    std::size_t leaf_obj_count = (*leaf_node)->getSize();

    while (leaf_obj_count >= this->max_objs_per_leaf_ && depth_mask) {
      // index to branch child
      unsigned char child_idx = key.getChildIdxWithDepthMask(depth_mask * 2);

      expandLeafNode(leaf_node, parent_branch_of_leaf_node, child_idx, depth_mask);

      depth_mask =
          this->createLeafRecursive(key, this->depth_mask_, this->root_node_, leaf_node, parent_branch_of_leaf_node);
      leaf_obj_count = (*leaf_node)->getSize();
    }
  }

  (*leaf_node)->addPointIndex(point_idx_arg);
  if constexpr (has_add_point_v<LeafContainerT, PointT>) { (*leaf_node)->addPoint(point); }
}

template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void jpcc::octree::JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::expandLeafNode(
    LeafNode* leaf_node, BranchNode* parent_branch, unsigned char child_idx, uindex_t depth_mask) {
  if (depth_mask) {
    // get amount of objects in leaf container
    std::size_t leaf_obj_count = (*leaf_node)->getSize();

    // copy leaf data
    Indices leafIndices;
    leafIndices.reserve(leaf_obj_count);

    (*leaf_node)->getPointIndices(leafIndices);

    // delete current leaf node
    this->deleteBranchChild(*parent_branch, child_idx);
    this->leaf_count_--;

    // create new branch node
    BranchNode* childBranch = this->createBranchChild(*parent_branch, child_idx);
    this->branch_count_++;

    // add data to new branch
    OctreeKey new_index_key;

    for (const auto& leafIndex : leafIndices) {
      const PointT& point_from_index = (*this->input_)[leafIndex];
      // generate key
      this->genOctreeKeyforPoint(point_from_index, new_index_key);

      LeafNode*   newLeaf;
      BranchNode* newBranchParent;
      this->createLeafRecursive(new_index_key, depth_mask, childBranch, newLeaf, newBranchParent);

      (*newLeaf)->addPointIndex(leafIndex);
      if constexpr (has_add_point_v<LeafContainerT, PointT>) { (*newLeaf)->addPoint(point_from_index); }
    }
  }
}

}  // namespace jpcc::octree