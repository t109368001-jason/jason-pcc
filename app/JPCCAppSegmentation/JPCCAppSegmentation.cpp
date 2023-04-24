#include <chrono>
#include <fstream>
#include <vector>

#include <boost/log/trivial.hpp>

#include <jpcc/common/JPCCContext.h>
#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/io/PlyIO.h>
#include <jpcc/segmentation/JPCCSegmentationAdapter.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::io;
using namespace jpcc::segmentation;

void encode(const AppParameter& parameter) {
  DatasetReader::Ptr    reader          = newReader(parameter.inputReader, parameter.inputDataset);
  JPCCSegmentation::Ptr gmmSegmentation = JPCCSegmentationAdapter::build(
      parameter.jpccGmmSegmentation, static_cast<int>(parameter.inputDataset.getStartFrameNumber()));

  {  // build gaussian mixture model
    GroupOfFrame frames;
    size_t       frameNumber    = parameter.inputDataset.getStartFrameNumber();
    const size_t endFrameNumber = parameter.inputDataset.getEndFrameNumber();
    while (!gmmSegmentation->isBuilt() && frameNumber < endFrameNumber) {
      size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
      reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);

      GroupOfPclFrame<pcl::PointXYZINormal> pclFrames;
      for (const auto& frame : frames) {
        pclFrames.push_back(frame->toPcl<pcl::PointXYZINormal>());
      }

      gmmSegmentation->appendTrainSamplesAndBuild(frames, pclFrames, parameter.parallel);

      frameNumber += groupOfFramesSize;
    }
  }

  GroupOfFrame frames;
  size_t       frameNumber    = parameter.inputDataset.getStartFrameNumber();
  const size_t endFrameNumber = parameter.inputDataset.getEndFrameNumber();
  while (frameNumber < endFrameNumber) {
    size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);

    GroupOfPclFrame<pcl::PointXYZINormal> pclFrames;
    GroupOfFrame                          dynamicFrames;
    GroupOfFrame                          staticAddedFrames;
    GroupOfFrame                          staticRemovedFrames;
    for (const auto& frame : frames) {
      pclFrames.push_back(frame->toPcl<pcl::PointXYZINormal>());
      auto dynamicFrame       = make_shared<Frame>();
      auto staticAddedFrame   = make_shared<Frame>();
      auto staticRemovedFrame = make_shared<Frame>();
      dynamicFrame->setFrameNumber(frame->getFrameNumber());
      staticAddedFrame->setFrameNumber(frame->getFrameNumber());
      staticRemovedFrame->setFrameNumber(frame->getFrameNumber());
      dynamicFrame->addRemoveAttributes(*frame);
      staticAddedFrame->addRemoveAttributes(*frame);
      staticRemovedFrame->addRemoveAttributes(*frame);
      dynamicFrames.push_back(dynamicFrame);
      staticAddedFrames.push_back(staticAddedFrame);
      staticRemovedFrames.push_back(staticRemovedFrame);
    }
    gmmSegmentation->segmentation(frames, pclFrames, dynamicFrames, staticAddedFrames, staticRemovedFrames);

    savePly(dynamicFrames, parameter.outputDataset.getFilePath(0), parameter.parallel, false);
    savePly(staticAddedFrames, parameter.outputDataset.getFilePath(1), parameter.parallel, false);
    savePly(staticRemovedFrames, parameter.outputDataset.getFilePath(2), parameter.parallel, false);

    frameNumber += groupOfFramesSize;
  }
}

int main(int argc, char* argv[]) {
  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Segmentation Start";

  AppParameter parameter;
  try {
    ParameterParser pp;
    pp.add(parameter);
    if (!pp.parse(argc, argv)) {
      return 1;
    }
    BOOST_LOG_TRIVIAL(info) << parameter;
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
    return 1;
  }

  try {
    // Timers to count elapsed wall/user time
    Stopwatch clockWall;
    Stopwatch clockUser;

    clockWall.start();
    encode(parameter);
    clockWall.stop();

    auto totalWall = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUser = duration_cast<milliseconds>(clockUser.count()).count();
    BOOST_LOG_TRIVIAL(info) << "Processing time (wall): " << static_cast<float>(totalWall) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Processing time (user): " << static_cast<float>(totalUser) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Peak memory: " << getPeakMemory() << " KB";
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
  }

  BOOST_LOG_TRIVIAL(info) << "JPCC App Segmentation End";
  return 0;
}