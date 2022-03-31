#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBufBase.h>

#include <test_data/octree/TestData.h>

namespace jpcc::octree {

ChildPattern getTestChildPattern(BufferIndex bufferSelector);

FramePtr<Point> getTestCloud(BufferIndex bufferSelector);

}  // namespace jpcc::octree
