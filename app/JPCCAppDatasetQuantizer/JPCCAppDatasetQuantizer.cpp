#include <chrono>
#include <execution>
#include <iostream>
#include <vector>

#include <boost/log/trivial.hpp>
#include <boost/range/counting_range.hpp>

#include <jpcc/common/ParameterParser.h>
#include <jpcc/io/Reader.h>
#include <jpcc/io/PlyIO.h>
#include <pcl/octree/octree_pointcloud.h>

#include "AppParameter.h"

using namespace std;
using namespace std::chrono;
using namespace jpcc;
using namespace jpcc::io;

void quantize(const AppParameter& parameter, Stopwatch& clock) {
  const typename DatasetReader::Ptr reader = newReader(parameter.inputReader, parameter.inputDataset);

  GroupOfFrame frames;
  size_t       frameNumber    = parameter.inputDataset.getStartFrameNumber();
  const size_t endFrameNumber = parameter.inputDataset.getEndFrameNumber();
  while (frameNumber < endFrameNumber) {
    size_t groupOfFramesSize = std::min(parameter.groupOfFramesSize, endFrameNumber - frameNumber);
    clock.start();
    reader->loadAll(frameNumber, groupOfFramesSize, frames, parameter.parallel);
    clock.stop();

    GroupOfFrame outputFrames(frames.size());

    auto func = [&](const FramePtr& frame) {
      if (!frame) {
        return FramePtr{};
      }
      auto outputFrame = make_shared<Frame>();
      outputFrame->addRemoveAttributes(*frame);

      auto                                         cloud = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(parameter.qp);
      octree.setInputCloud(cloud);
      size_t oIndex = 0;
      for (size_t i = 0; i < frame->getPointCount(); i++) {
        PointType& point = (*frame)[i];
        if (!parameter.offset.empty()) {
          point[0] += static_cast<PointValueType>(parameter.offset[0]);
          point[1] += static_cast<PointValueType>(parameter.offset[1]);
          point[2] += static_cast<PointValueType>(parameter.offset[2]);
        }
        point[0] /= parameter.qp;
        point[1] /= parameter.qp;
        point[2] /= parameter.qp;
        THROW_IF_NOT(point[0] >= 0 && point[1] >= 0 && point[1] >= 0);
        const pcl::PointXYZ& p =
            pcl::PointXYZ(static_cast<float>(point[0]), static_cast<float>(point[1]), static_cast<float>(point[2]));
        if (!octree.isVoxelOccupiedAtPoint(p)) {
          octree.addPointToCloud(p, cloud);
          outputFrame->addPoint(point);
          if (frame->hasReflectances()) {
            outputFrame->setReflectance(oIndex, frame->getReflectance(i));
          }
          if (frame->hasNormals()) {
            outputFrame->setNormal(oIndex, frame->getNormal(i));
          }
          oIndex++;
        }
      }
      return outputFrame;
    };

    if (!parameter.parallel) {
      for (size_t i = 0; i < frames.size(); i++) {
        outputFrames[i] = func(frames[i]);
      }
    } else {
      const auto range = boost::counting_range<size_t>(0, frames.size());
      std::for_each(std::execution::par, range.begin(), range.end(),
                    [&](const size_t& i) {  //
                      outputFrames[i] = func(frames[i]);
                    });
    }

    savePly(frames, parameter.outputDataset.getFilePath(), parameter.parallel, false);

    frameNumber += groupOfFramesSize;
  }
}

int main(int argc, char* argv[]) {
  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Quantizer Start";

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
    quantize(parameter, clockUser);
    clockWall.stop();

    auto totalWall = duration_cast<milliseconds>(clockWall.count()).count();
    auto totalUser = duration_cast<milliseconds>(clockUser.count()).count();
    BOOST_LOG_TRIVIAL(info) << "Processing time (wall): " << static_cast<float>(totalWall) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Processing time (user): " << static_cast<float>(totalUser) / 1000.0 << " s";
    BOOST_LOG_TRIVIAL(info) << "Peak memory: " << getPeakMemory() << " KB";
  } catch (exception& e) {
    BOOST_LOG_TRIVIAL(error) << e.what();
  }

  BOOST_LOG_TRIVIAL(info) << "JPCC App Dataset Quantizer End";
  return 0;
}