#include <jpcc/metric/JPCCMetric.h>

#include <execution>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <boost/config.hpp>
#include <boost/log/trivial.hpp>
#include <boost/range/counting_range.hpp>

#include <KDTreeVectorOfVectorsAdaptor.h>

using namespace std;

namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCMetric::JPCCMetric(const JPCCMetricParameter& parameter) :
    parameter_(parameter),
    frameNumberSet_(),
    pointsNameSet_(),
    bytesNameSet_(),
    c2cMSENameSet_(),
    c2cPSNRNameSet_(),
    c2pMSENameSet_(),
    c2pPSNRNameSet_(),
    clockNameSet_(),
    pointsMapMap_(),
    bytesMapMap_(),
    c2cMSEMapMap_(),
    c2cPSNRMapMap_(),
    c2pMSEMapMap_(),
    c2pPSNRMapMap_(),
    clockMapMap_() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addPoints(const std::string& name, const FramePtr& frame, bool addBytes) {
  const auto& points = frame->getPointCount();
  if (points == 0) {
    return;
  }
  frameNumberSet_.insert(frame->getFrameNumber());
  pointsNameSet_.insert(name);
  pointsMapMap_[frame->getFrameNumber()][name] += points;
  BOOST_LOG_TRIVIAL(info) << "name=" << name << ", "
                          << "frameNumber_=" << frame->getFrameNumber() << ", "
                          << "points=" << points;
  if (addBytes) {
    this->addBytes(name, frame->getFrameNumber(), points * sizeof(PointValueType) * 3);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addPoints(const std::string& name, const GroupOfFrame& frames, bool addBytes) {
  for (const auto& frame : frames) {
    this->addPoints(name, frame, addBytes);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addBytes(const std::string& name, FrameNumber frameNumber, const uint64_t bytes) {
  if (bytes == 0) {
    return;
  }
  BOOST_LOG_TRIVIAL(info) << "name=" << name << ", "
                          << "frameNumber_=" << frameNumber << ", "
                          << "bytes=" << bytes;
  frameNumberSet_.insert(frameNumber);
  bytesNameSet_.insert(name);
  bytesMapMap_[frameNumber][name] = bytes;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addBytes(const std::string&                    name,
                          FrameNumber                           firstFrameNumber,
                          const std::vector<std::vector<char>>& bytesVector) {
  for (size_t i = 0; i < bytesVector.size(); i++) {
    this->addBytes(name, firstFrameNumber + i, bytesVector[i].size());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addPSNR(const std::string& name, const FramePtr& frameA, const FramePtr& frameB) {
  double c2cMSE  = std::numeric_limits<double>::quiet_NaN();
  double c2cPSNR = std::numeric_limits<double>::quiet_NaN();
  double c2pMSE  = std::numeric_limits<double>::quiet_NaN();
  double c2pPSNR = std::numeric_limits<double>::quiet_NaN();

  computePSNR(frameA, frameB, c2cMSE, c2cPSNR, c2pMSE, c2pPSNR);

  frameNumberSet_.insert(frameA->getFrameNumber());
  c2cMSENameSet_.insert(name);
  c2cPSNRNameSet_.insert(name);
  c2pMSENameSet_.insert(name);
  c2pPSNRNameSet_.insert(name);
  c2cMSEMapMap_[frameA->getFrameNumber()][name]  = c2cMSE;
  c2cPSNRMapMap_[frameA->getFrameNumber()][name] = c2cPSNR;
  c2pMSEMapMap_[frameA->getFrameNumber()][name]  = c2pMSE;
  c2pPSNRMapMap_[frameA->getFrameNumber()][name] = c2pPSNR;
  BOOST_LOG_TRIVIAL(info) << "name=" << name << ", "
                          << "frameNumberA=" << frameA->getFrameNumber() << ", "
                          << "frameNumberB=" << frameB->getFrameNumber() << ", "
                          << "pointsA=" << frameA->getPointCount() << ", "
                          << "pointsB=" << frameB->getPointCount() << ", "
                          << "c2cMSE=" << c2cMSE << ", "
                          << "c2cPSNR=" << c2cPSNR << ", "
                          << "c2pMSE=" << c2pMSE << ", "
                          << "c2pPSNR=" << c2pPSNR;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addPSNR(const std::string&  name,
                         const GroupOfFrame& framesA,
                         const GroupOfFrame& framesB,
                         const bool          parallel) {
  assert(framesA.size() == framesB.size());
  if (!parallel) {
    for (size_t i = 0; i < framesA.size(); i++) {
      this->addPSNR(name, framesA[i], framesB[i]);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, framesA.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->addPSNR(name, framesA[i], framesB[i]);
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::copyNormalToReconstruct(const FramePtr& frame, const FramePtr& reconstructFrame) {
  if (frame->getPointCount() == 0 || reconstructFrame->getPointCount() == 0) {
    return;
  }
  size_t                                      K = 1;
  std::vector<size_t>                         pointIdxKNNSearch(K);
  std::vector<double>                         pointKNNSquaredDistance(K);
  nanoflann::KNNResultSet<double>             resultSet(K);
  KDTreeVectorOfVectorsAdaptor<Frame, double> kdtree(3, *frame, 10);

  reconstructFrame->addNormals();
  for (size_t i = 0; i < reconstructFrame->getPointCount(); i++) {
    auto&                 point       = (*reconstructFrame)[i];
    std::array<double, 3> pointDouble = {static_cast<double>(point[0]), static_cast<double>(point[1]),
                                         static_cast<double>(point[2])};
    resultSet.init(&pointIdxKNNSearch[0], &pointKNNSquaredDistance[0]);
    bool ret = kdtree.index->findNeighbors(resultSet, &pointDouble[0], nanoflann::SearchParams(10));
    THROW_IF_NOT(ret);
    reconstructFrame->setNormal(i, frame->getNormal(pointIdxKNNSearch.front()));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::copyNormalToReconstruct(const GroupOfFrame& frames,
                                         const GroupOfFrame& reconstructFrames,
                                         const bool          parallel) {
  assert(frames.size() == reconstructFrames.size());
  if (!parallel) {
    for (size_t i = 0; i < frames.size(); i++) {
      const FramePtr& frame            = frames[i];
      const FramePtr& reconstructFrame = reconstructFrames[i];
      this->copyNormalToReconstruct(frame, reconstructFrame);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    const FramePtr& frame            = frames[i];
                    const FramePtr& reconstructFrame = reconstructFrames[i];
                    this->copyNormalToReconstruct(frame, reconstructFrame);
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::copyNormalToReconstruct(const JPCCContext& encoderContext,
                                         const JPCCContext& decoderContext,
                                         const bool         parallel) {
  this->copyNormalToReconstruct(encoderContext.getDynamicFrames(), decoderContext.getDynamicFrames(), parallel);
  if (encoderContext.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    this->copyNormalToReconstruct(encoderContext.getStaticFrames(), decoderContext.getStaticFrames(), parallel);
  }
  if (encoderContext.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    this->copyNormalToReconstruct(encoderContext.getStaticAddedFrames(), decoderContext.getStaticAddedFrames(),
                                  parallel);
    this->copyNormalToReconstruct(encoderContext.getStaticRemovedFrames(), decoderContext.getStaticRemovedFrames(),
                                  parallel);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::computePSNR(const FramePtr& frameA,
                             const FramePtr& frameB,
                             double&         c2cMSE,
                             double&         c2cPSNR,
                             double&         c2pMSE,
                             double&         c2pPSNR) const {
  double sseC2c = 0;
  double sseC2p = 0;

  size_t                          K = 1;
  std::vector<size_t>             pointIdxKNNSearch(K);
  std::vector<double>             pointKNNSquaredDistance(K);
  nanoflann::KNNResultSet<double> resultSet(K);

  KDTreeVectorOfVectorsAdaptor<Frame, double> kdtree(3, *frameB, 10);

  for (size_t indexA = 0; indexA < frameA->getPointCount(); indexA++) {
    auto&        pointA       = (*frameA)[indexA];
    Vec3<double> pointADouble = {static_cast<double>(pointA[0]), static_cast<double>(pointA[1]),
                                 static_cast<double>(pointA[2])};
    // For point 'i' in A, find its nearest neighbor in B. store it in 'j'
    resultSet.init(&pointIdxKNNSearch[0], &pointKNNSquaredDistance[0]);
    bool ret = kdtree.index->findNeighbors(resultSet, &pointADouble[0], nanoflann::SearchParams(10));
    THROW_IF_NOT(ret);

    const auto& pointB = (*frameB)[pointIdxKNNSearch.front()];

    // Compute point-to-point, which should be equal to sqrt( dist[0] )
    const double distProjC2c = pointKNNSquaredDistance.front();

    // Compute point-to-plane, normals in B will be used for point-to-plane
    double distProjC2p;
    if (frameB->hasNormals()) {
      std::array<float, 3> err = {static_cast<float>(pointA[0] - pointB[0]), static_cast<float>(pointA[1] - pointB[1]),
                                  static_cast<float>(pointA[2] - pointB[2])};
      NormalType           normalB = frameB->getNormal(pointIdxKNNSearch.front());
      distProjC2p                  = err[0] * normalB[0] + err[1] * normalB[1] + err[2] * normalB[2];
      distProjC2p *= distProjC2p;
    } else {
      distProjC2p = distProjC2c;
    }

    // mean square distance
    sseC2c += distProjC2c;
    sseC2p += distProjC2p;
  }

  c2cMSE  = sseC2c / static_cast<float>(frameA->getPointCount());
  c2cPSNR = 10 * log10((3 * pow(parameter_.maximumValue, 2)) / c2cMSE);
  c2pMSE  = sseC2p / static_cast<float>(frameA->getPointCount());
  c2pPSNR = 10 * log10((3 * pow(parameter_.maximumValue, 2)) / c2pMSE);
}

//////////////////////////////////////////////////////////////////////////////////////////////
ScopeStopwatch JPCCMetric::start(const std::string& name, const FrameNumber frameNumber) {
  frameNumberSet_.insert(frameNumber);
  clockNameSet_.insert(name);
  Stopwatch& clock = clockMapMap_[frameNumber][name];
  return {clock};
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::writeAndShow() {
  std::map<std::string, uint64_t> pointsSumMap_;
  std::map<std::string, uint64_t> bytesSumMap_;
  std::map<std::string, double>   c2cMSESumMap_;
  std::map<std::string, double>   c2cPSNRSumMap_;
  std::map<std::string, double>   c2pMSESumMap_;
  std::map<std::string, double>   c2pPSNRSumMap_;
  std::map<std::string, double>   clockSumMap_;

  std::ofstream metricCSV(parameter_.outputCSVFolderPath / "metric.csv");
  {  // write header
    metricCSV << "Frame Number,";
    for (const auto& name : pointsNameSet_) {
      metricCSV << name << " (points),";
    }
    for (const auto& name : bytesNameSet_) {
      metricCSV << name << " (bytes),";
    }
    for (const auto& name : c2cMSENameSet_) {
      metricCSV << name << " (c2c)(mm^2),";
    }
    for (const auto& name : c2cPSNRNameSet_) {
      metricCSV << name << " (c2c)(db),";
    }
    for (const auto& name : c2pMSENameSet_) {
      metricCSV << name << " (c2p)(mm^2),";
    }
    for (const auto& name : c2pPSNRNameSet_) {
      metricCSV << name << " (c2p)(db),";
    }
    for (const auto& name : clockNameSet_) {
      metricCSV << name << " (s),";
    }
    metricCSV << std::endl;
  }
  for (const auto& frameNumber : frameNumberSet_) {  // write rows
    auto& pointsMap  = pointsMapMap_[frameNumber];
    auto& bytesMap   = bytesMapMap_[frameNumber];
    auto& c2cMSEMap  = c2cMSEMapMap_[frameNumber];
    auto& c2cPSNRMap = c2cPSNRMapMap_[frameNumber];
    auto& c2pMSEMap  = c2pMSEMapMap_[frameNumber];
    auto& c2pPSNRMap = c2pPSNRMapMap_[frameNumber];
    auto& clockMap   = clockMapMap_[frameNumber];
    metricCSV << frameNumber << ",";
    for (const auto& name : pointsNameSet_) {
      const auto& points = pointsMap[name];
      metricCSV << points << ",";
      pointsSumMap_[name] += points;
    }
    for (const auto& name : bytesNameSet_) {
      const auto& bytes = bytesMap[name];
      metricCSV << bytes << ",";
      bytesSumMap_[name] += bytes;
    }
    for (const auto& name : c2cMSENameSet_) {
      const auto& mse = c2cMSEMap[name];
      metricCSV << mse << ",";
      c2cMSESumMap_[name] += mse;
    }
    for (const auto& name : c2cPSNRNameSet_) {
      const auto& psnr = c2cPSNRMap[name];
      metricCSV << psnr << ",";
      c2cPSNRSumMap_[name] += psnr;
    }
    for (const auto& name : c2pMSENameSet_) {
      const auto& mse = c2pMSEMap[name];
      metricCSV << mse << ",";
      c2pMSESumMap_[name] += mse;
    }
    for (const auto& name : c2pPSNRNameSet_) {
      const auto& psnr = c2pPSNRMap[name];
      metricCSV << psnr << ",";
      c2pPSNRSumMap_[name] += psnr;
    }
    for (const auto& name : clockNameSet_) {
      double duration = static_cast<double>(clockMap[name].count().count()) / 1000000000.0;
      metricCSV << duration << ",";
      clockSumMap_[name] += duration;
    }
    metricCSV << std::endl;
  }
  std::ofstream metricSummaryCSV(parameter_.outputCSVFolderPath / "metric-summary.csv");
  metricSummaryCSV << "Platform," << BOOST_PLATFORM << endl;
#if defined(BOOST_CXX_VERSION)
  metricSummaryCSV << "C++ Version," << BOOST_CXX_VERSION << endl;
#elif defined(__cplusplus)
  metricSummaryCSV << "C++ Version," << __cplusplus << endl;
#endif
  metricSummaryCSV << "Compiler," << BOOST_COMPILER << endl;

  BOOST_LOG_TRIVIAL(info) << "Environment Info:";
  BOOST_LOG_TRIVIAL(info) << "  Platform          :" << BOOST_PLATFORM;
#if defined(BOOST_CXX_VERSION)
  BOOST_LOG_TRIVIAL(info) << "  C++ Version       :" << BOOST_CXX_VERSION;
#elif defined(__cplusplus)
  BOOST_LOG_TRIVIAL(info) << "  C++ Version       :" << __cplusplus;
#endif
  BOOST_LOG_TRIVIAL(info) << "  Compiler          :" << BOOST_COMPILER;

  unsigned long frameCount = frameNumberSet_.size();
  metricSummaryCSV << "Frame Count," << frameCount << endl;

  int maxNameLength = 0;
  for (const auto& name : pointsNameSet_) {
    maxNameLength = max(maxNameLength, static_cast<int>(name.size()));
  }
  for (const auto& name : bytesNameSet_) {
    maxNameLength = max(maxNameLength, static_cast<int>(name.size()));
  }
  for (const auto& name : c2cMSENameSet_) {
    maxNameLength = max(maxNameLength, static_cast<int>(name.size()));
  }
  for (const auto& name : c2cPSNRNameSet_) {
    maxNameLength = max(maxNameLength, static_cast<int>(name.size()));
  }
  for (const auto& name : c2pMSENameSet_) {
    maxNameLength = max(maxNameLength, static_cast<int>(name.size()));
  }
  for (const auto& name : c2pPSNRNameSet_) {
    maxNameLength = max(maxNameLength, static_cast<int>(name.size()));
  }
  for (const auto& name : clockNameSet_) {
    maxNameLength = max(maxNameLength, static_cast<int>(name.size()));
  }
  BOOST_LOG_TRIVIAL(info) << "";
  BOOST_LOG_TRIVIAL(info) << "Metric:";
  BOOST_LOG_TRIVIAL(info) << "  Points:";
  for (const auto& [name, points] : pointsSumMap_) {  //
    metricSummaryCSV << name << " (points)," << points << endl;
    BOOST_LOG_TRIVIAL(info) << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << points
                            << " points";
  }
  BOOST_LOG_TRIVIAL(info) << "  Bytes:";
  for (const auto& [name, bytes] : bytesSumMap_) {  //
    metricSummaryCSV << name << " (bytes)," << bytes << endl;
    BOOST_LOG_TRIVIAL(info) << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << bytes
                            << " bytes";
  }
  BOOST_LOG_TRIVIAL(info) << "  MSE (c2c):";
  for (const auto& [name, mse] : c2cMSESumMap_) {  //
    metricSummaryCSV << name << " (mm^2)," << mse / static_cast<double>(frameCount) << endl;
    BOOST_LOG_TRIVIAL(info) << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": "
                            << mse / static_cast<double>(frameCount) << " mm^2";
  }
  BOOST_LOG_TRIVIAL(info) << "  PSNR (c2c):";
  for (const auto& [name, psnr] : c2cPSNRSumMap_) {  //
    metricSummaryCSV << name << " (db)," << psnr / static_cast<double>(frameCount) << endl;
    BOOST_LOG_TRIVIAL(info) << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": "
                            << psnr / static_cast<double>(frameCount) << " db";
  }
  BOOST_LOG_TRIVIAL(info) << "  MSE (c2p):";
  for (const auto& [name, mse] : c2pMSESumMap_) {  //
    metricSummaryCSV << name << " (mm^2)," << mse / static_cast<double>(frameCount) << endl;
    BOOST_LOG_TRIVIAL(info) << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": "
                            << mse / static_cast<double>(frameCount) << " mm^2";
  }
  BOOST_LOG_TRIVIAL(info) << "  PSNR (c2p):";
  for (const auto& [name, psnr] : c2pPSNRSumMap_) {  //
    metricSummaryCSV << name << " (db)," << psnr / static_cast<double>(frameCount) << endl;
    BOOST_LOG_TRIVIAL(info) << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": "
                            << psnr / static_cast<double>(frameCount) << " db";
  }
  BOOST_LOG_TRIVIAL(info) << "  Processing time:";
  for (const auto& [name, duration] : clockSumMap_) {
    metricSummaryCSV << name << " (s)," << duration << endl;
    BOOST_LOG_TRIVIAL(info) << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": "
                            << duration << " s";
  }
  metricSummaryCSV << "Peek memory (KB)," << getPeakMemory() << endl;
  BOOST_LOG_TRIVIAL(info) << "Peak memory: " << getPeakMemory() << " KB\n";
}

}  // namespace jpcc::metric