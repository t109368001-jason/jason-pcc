#include <array>
#include <chrono>
#include <iostream>
#include <vector>

#include <pcl/io/ply_io.h>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/process/JPCCNormalEstimation.h>
#include <jpcc/process/PreProcessor.h>
#include <jpcc/process/Process.h>
#include <jpcc/octree/OctreeNBuf.h>

#include "AppParameter.h"

#include <jpcc/octree/JPCCOctreePointCloud.h>

using namespace std;
using namespace std::chrono;
using namespace pcl;
using namespace pcl::octree;
using namespace pcc;
using namespace pcc::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::octree;
using namespace jpcc::process;

#define BUFFER_SIZE 100

using LeafContainerT    = OctreeContainerPointIndices;
using BranchContainerT  = OctreeContainerEmpty;
using OctreeNBufT       = OctreeNBuf<BUFFER_SIZE, LeafContainerT, BranchContainerT>;
using OctreePointCloudT = JPCCOctreePointCloud<PointXYZINormal, LeafContainerT, BranchContainerT, OctreeNBufT>;

void backgroundGenerator(const AppParameter& parameter, StopwatchUserTime& clock) {
  size_t frameNumber = parameter.dataset.getStartFrameNumber();

  const DatasetReader::Ptr   reader = newReader(parameter.reader, parameter.dataset);
  const PreProcessor         preProcessor(parameter.preProcess);
  const JPCCNormalEstimation normalEstimation(parameter.jpccNormalEstimation);

  auto staticCloud_ = jpcc::make_shared<Frame>();
  {
    BufferIndex                  bufferIndex = 0;
    array<FramePtr, BUFFER_SIZE> frameBuffer;
    OctreePointCloudT            octreePointCloud(parameter.filterResolution);

    GroupOfFrame frames;

    clock.start();
    reader->loadAll(frameNumber, BUFFER_SIZE, frames, parameter.parallel);
    preProcessor.process(frames, nullptr, parameter.parallel);
    //    normalEstimation.computeInPlaceAll(frames, parameter.parallel);
    clock.stop();

    for (size_t i = 0; i < frames.size(); i++) {
      if (i != 0) { bufferIndex = (bufferIndex + 1) % BUFFER_SIZE; }
      frameBuffer.at(bufferIndex) = frames.at(i);
      octreePointCloud.setFrame(bufferIndex, frameBuffer.at(bufferIndex));
    }

    JPCCOctreePointCloud<PointXYZINormal, OctreeContainerPointIndex, BranchContainerT,
                         OctreeNBuf<1, OctreeContainerPointIndex, BranchContainerT>>
        staticOctree(parameter.backgroundResolution);
    staticOctree.setInputCloud(staticCloud_);
    OctreeNBufT::LeafBranchCallback callback = [&](const ChildIndex               childIndex,
                                                   const OctreeNBufT::BranchNode& branchNode) {
      const OctreeNBufT::BufferPattern& bufferPattern = branchNode.getBufferPattern(childIndex);
      if ((float)bufferPattern.count() > BUFFER_SIZE * parameter.backgroundThreshold) {
        for (BufferIndex _bufferIndex = 0; _bufferIndex < BUFFER_SIZE; _bufferIndex++) {
          if (branchNode.hasChild(_bufferIndex, childIndex)) {
            Indices& _indices = dynamic_cast<OctreeNBufT::LeafNode*>(branchNode.getChildPtr(_bufferIndex, childIndex))
                                    ->getContainer()
                                    .getPointIndicesVector();
            for_each(_indices.begin(), _indices.end(), [&](const auto& index) {
              PointXYZINormal point;
              pcl::copyPoint(frameBuffer.at(_bufferIndex)->at(index), point);
              if (!staticOctree.isVoxelOccupiedAtPoint(point)) { staticOctree.addPointToCloud(point, staticCloud_); }
            });
          }
        }
      }
    };

    octreePointCloud.forEachLeafBranch(callback);
  }

  pcl::io::savePLYFile(parameter.getOutputPath(), *staticCloud_);
}

int main(int argc, char* argv[]) {
  cout << "JPCC App Background Generator Start" << endl;

  AppParameter parameter;
  try {
    ParameterParser pp;
    pp.add(parameter);
    if (!pp.parse(argc, argv)) { return 1; }
    cout << parameter << endl;
  } catch (exception& e) {
    cerr << e.what() << endl;
    return 1;
  }

  // Timers to count elapsed wall/user time
  Stopwatch<steady_clock> clockWall;
  StopwatchUserTime       clockUser;

  clockWall.start();
  backgroundGenerator(parameter, clockUser);
  clockWall.stop();

  auto totalWall      = duration_cast<milliseconds>(clockWall.count()).count();
  auto totalUserSelf  = duration_cast<milliseconds>(clockUser.self.count()).count();
  auto totalUserChild = duration_cast<milliseconds>(clockUser.children.count()).count();
  cout << "Processing time (wall): " << (float)totalWall / 1000.0 << " s\n";
  cout << "Processing time (user.self): " << (float)totalUserSelf / 1000.0 << " s\n";
  cout << "Processing time (user.children): " << (float)totalUserChild / 1000.0 << " s\n";
  cout << "Peak memory: " << getPeakMemory() << " KB\n";

  cout << "JPCC App Background Generator End" << endl;
  return 0;
}