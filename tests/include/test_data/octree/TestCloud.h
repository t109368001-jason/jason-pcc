#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBuf.h>

#include <test_data/octree/TestData.h>

namespace jpcc::octree {

ChildPattern getTestChildPattern(BufferIndex bufferSelector);

FramePtr getTestCloud(BufferIndex bufferSelector);

std::vector<size_t> getCounts();

}  // namespace jpcc::octree
