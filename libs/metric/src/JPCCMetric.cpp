#include <jpcc/metric/JPCCMetric.h>

#include <execution>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <boost/config.hpp>
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
    clockMapMap_() {}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addPoints(const std::string& name, const FramePtr& frame, bool addBytes) {
  const auto& points = frame->getPointCount();
  if (points == 0) { return; }
  frameNumberSet_.insert(frame->getFrameNumber());
  pointsNameSet_.insert(name);
  pointsMapMap_[frame->getFrameNumber()][name] += points;
  std::cout << __FUNCTION__ << "() "
            << "name=" << name << ", "
            << "frameNumber_=" << frame->getFrameNumber() << ", "
            << "points=" << points << std::endl;
  if (addBytes) { this->addBytes(name, frame->getFrameNumber(), points * sizeof(PointValueType) * 3); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addPoints(const std::string& name, const GroupOfFrame& frames, bool addBytes) {
  for (const auto& frame : frames) { this->addPoints(name, frame, addBytes); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addBytes(const std::string& name, FrameNumber frameNumber, const uint64_t bytes) {
  if (bytes == 0) { return; }
  cout << __FUNCTION__ << "() "
       << "name=" << name << ", "
       << "frameNumber_=" << frameNumber << ", "
       << "bytes=" << bytes << std::endl;
  frameNumberSet_.insert(frameNumber);
  bytesNameSet_.insert(name);
  bytesMapMap_[frameNumber][name] = bytes;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addBytes(const std::string&                    name,
                          FrameNumber                           firstFrameNumber,
                          const std::vector<std::vector<char>>& bytesVector) {
  for (size_t i = 0; i < bytesVector.size(); i++) { this->addBytes(name, firstFrameNumber + i, bytesVector[i].size()); }
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
  std::cout << __FUNCTION__ << "() "
            << "name=" << name << ", "
            << "frameNumberA=" << frameA->getFrameNumber() << ", "
            << "frameNumberB=" << frameB->getFrameNumber() << ", "
            << "pointsA=" << frameA->getPointCount() << ", "
            << "pointsB=" << frameB->getPointCount() << ", "
            << "c2cMSE=" << c2cMSE << ", "
            << "c2cPSNR=" << c2cPSNR << ", "
            << "c2pMSE=" << c2pMSE << ", "
            << "c2pPSNR=" << c2pPSNR << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCMetric::addPSNR(const std::string&  name,
                         const GroupOfFrame& framesA,
                         const GroupOfFrame& framesB,
                         const bool          parallel) {
  assert(framesA.size() == framesB.size());
  if (!parallel) {
    for (size_t i = 0; i < framesA.size(); i++) { this->addPSNR(name, framesA[i], framesB[i]); }
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
  if (frame->getPointCount() == 0 || reconstructFrame->getPointCount() == 0) { return; }
  size_t                                      K = 1;
  std::vector<size_t>                         pointIdxKNNSearch(K);
  std::vector<double>                         pointKNNSquaredDistance(K);
  nanoflann::KNNResultSet<double>             resultSet(K);
  KDTreeVectorOfVectorsAdaptor<Frame, double> kdtree(3, *frame, 10);

  reconstructFrame->addNormals();
  for (size_t i = 0; i < reconstructFrame->getPointCount(); i++) {
    auto&                 point       = (*reconstructFrame)[i];
    std::array<double, 3> pointDouble = {(double)point[0], (double)point[1], (double)point[2]};
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
void JPCCMetric::copyNormalToReconstruct(const JPCCContext& context, const bool parallel) {
  this->copyNormalToReconstruct(context.getDynamicFrames(), context.getDynamicReconstructFrames(), parallel);
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    this->copyNormalToReconstruct(context.getStaticFrames(), context.getStaticReconstructFrames(), parallel);
  }
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    this->copyNormalToReconstruct(context.getStaticAddedFrames(), context.getStaticAddedReconstructFrames(), parallel);
    this->copyNormalToReconstruct(context.getStaticRemovedFrames(), context.getStaticRemovedReconstructFrames(),
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
    Vec3<double> pointADouble = {(double)pointA[0], (double)pointA[1], (double)pointA[2]};
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
      PointType  err     = {pointA[0] - pointB[0], pointA[1] - pointB[1], pointA[2] - pointB[2]};
      NormalType normalB = frameB->getNormal(pointIdxKNNSearch.front());
      distProjC2p        = err[0] * normalB[0] + err[1] * normalB[1] + err[2] * normalB[2];
      distProjC2p *= distProjC2p;
    } else {
      distProjC2p = distProjC2c;
    }

    // mean square distance
    sseC2c += distProjC2c;
    sseC2p += distProjC2p;
  }

  c2cMSE  = sseC2c / float(frameA->getPointCount());
  c2cPSNR = 10 * log10((3 * pow(parameter_.maximumValue, 2)) / c2cMSE);
  c2pMSE  = sseC2p / float(frameA->getPointCount());
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
    for (const auto& name : pointsNameSet_) { metricCSV << name << " (points),"; }
    for (const auto& name : bytesNameSet_) { metricCSV << name << " (bytes),"; }
    for (const auto& name : c2cMSENameSet_) { metricCSV << name << " (c2c)(mm^2),"; }
    for (const auto& name : c2cPSNRNameSet_) { metricCSV << name << " (c2c)(db),"; }
    for (const auto& name : c2pMSENameSet_) { metricCSV << name << " (c2p)(mm^2),"; }
    for (const auto& name : c2pPSNRNameSet_) { metricCSV << name << " (c2p)(db),"; }
    for (const auto& name : clockNameSet_) { metricCSV << name << " (s),"; }
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
      double duration = (double)clockMap[name].count().count() / 1000000000.0;
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

  cout << "Environment Info:" << endl;
  cout << "  Platform          :" << BOOST_PLATFORM << endl;
#if defined(BOOST_CXX_VERSION)
  cout << "  C++ Version       :" << BOOST_CXX_VERSION << endl;
#elif defined(__cplusplus)
  cout << "  C++ Version       :" << __cplusplus << endl;
#endif
  cout << "  Compiler          :" << BOOST_COMPILER << endl;

  unsigned long frameCount = frameNumberSet_.size();
  metricSummaryCSV << "Frame Count," << frameCount << endl;

  int maxNameLength = 0;
  for (const auto& name : pointsNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : bytesNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : c2cMSENameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : c2cPSNRNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : c2pMSENameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : c2pPSNRNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  for (const auto& name : clockNameSet_) { maxNameLength = max(maxNameLength, (int)name.size()); }
  cout << endl;
  cout << "Metric:" << endl;
  cout << "  Points:" << endl;
  for (const auto& [name, points] : pointsSumMap_) {  //
    metricSummaryCSV << name << " (points)," << points << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << points << " points"
         << endl;
  }
  cout << "  Bytes:" << endl;
  for (const auto& [name, bytes] : bytesSumMap_) {  //
    metricSummaryCSV << name << " (bytes)," << bytes << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << bytes << " bytes"
         << endl;
  }
  cout << "  MSE (c2c):" << endl;
  for (const auto& [name, mse] : c2cMSESumMap_) {  //
    metricSummaryCSV << name << " (mm^2)," << mse / (double)frameCount << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << mse / (double)frameCount
         << " mm^2" << endl;
  }
  cout << "  PSNR (c2c):" << endl;
  for (const auto& [name, psnr] : c2cPSNRSumMap_) {  //
    metricSummaryCSV << name << " (db)," << psnr / (double)frameCount << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << psnr / (double)frameCount
         << " db" << endl;
  }
  cout << "  MSE (c2p):" << endl;
  for (const auto& [name, mse] : c2pMSESumMap_) {  //
    metricSummaryCSV << name << " (mm^2)," << mse / (double)frameCount << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << mse / (double)frameCount
         << " mm^2" << endl;
  }
  cout << "  PSNR (c2p):" << endl;
  for (const auto& [name, psnr] : c2pPSNRSumMap_) {  //
    metricSummaryCSV << name << " (db)," << psnr / (double)frameCount << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << psnr / (double)frameCount
         << " db" << endl;
  }
  cout << "  Processing time:" << endl;
  for (const auto& [name, duration] : clockSumMap_) {
    metricSummaryCSV << name << " (s)," << duration << endl;
    cout << "    " << std::setfill(' ') << std::setw(maxNameLength) << name << left << ": " << duration << " s" << endl;
  }
  metricSummaryCSV << "Peek memory (KB)," << getPeakMemory() << endl;
  cout << "Peak memory: " << getPeakMemory() << " KB\n\n";
}

}  // namespace jpcc::metric