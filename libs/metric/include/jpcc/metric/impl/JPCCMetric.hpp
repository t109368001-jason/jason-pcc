#include <execution>

#include <boost/range/counting_range.hpp>

#include <pcl/kdtree/kdtree_flann.h>

namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPoints(const std::string& name, const FramePtr<PointT>& frame, bool addBytes) {
  const auto& points = frame->size();
  if (points == 0) { return; }
  uint32_t frameNumber = frame->header.seq;
  frameNumberSet_.insert(frameNumber);
  pointsNameSet_.insert(name);
  pointsMapMap_[frameNumber][name] += points;
  std::cout << __FUNCTION__ << "() "
            << "name=" << name << ", "
            << "frameNumber=" << frameNumber << ", "
            << "points=" << points << std::endl;
  if (addBytes) { this->addBytes(name, frameNumber, points * sizeof(float) * 3); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPoints(const std::string& name, const GroupOfFrame<PointT>& frames, bool addBytes) {
  for (const auto& frame : frames) { this->addPoints<PointT>(name, frame, addBytes); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPointsAndBytes(const std::string&       name,
                                   const FramePtr<PointT>&  frame,
                                   const std::vector<char>& encodedBytes) {
  if (encodedBytes.size() == 0) { return; }
  const auto& points = frame->size();
  if (points == 0) { return; }
  uint32_t frameNumber = frame->header.seq;
  frameNumberSet_.insert(frameNumber);
  pointsNameSet_.insert(name);
  pointsMapMap_[frameNumber][name] += points;
  bytesNameSet_.insert(name);
  bytesMapMap_[frameNumber][name] = encodedBytes.size();
  std::cout << __FUNCTION__ << "() "
            << "name=" << name << ", "
            << "frameNumber=" << frameNumber << ", "
            << "points=" << points << ", "
            << "bytes=" << encodedBytes.size() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPointsAndBytes(const std::string&                    name,
                                   const GroupOfFrame<PointT>&           frames,
                                   const std::vector<std::vector<char>>& encodedFramesBytes) {
  for (size_t i = 0; i < frames.size(); i++) {
    this->addPointsAndBytes<PointT>(name, frames[i], encodedFramesBytes[i]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointA, typename PointB>
void JPCCMetric::addPSNR(const std::string& name, const FramePtr<PointA>& frameA, const FramePtr<PointB>& frameB) {
  double c2cMSE  = 0;
  double c2cPSNR = 0;
  double c2pMSE  = 0;
  double c2pPSNR = 0;

  computePSNR<PointA, PointB>(frameA, frameB, c2cMSE, c2cPSNR, c2pMSE, c2pPSNR);

  frameNumberSet_.insert(frameA->header.seq);
  c2cMSENameSet_.insert(name);
  c2cPSNRNameSet_.insert(name);
  c2pMSENameSet_.insert(name);
  c2pPSNRNameSet_.insert(name);
  c2cMSEMapMap_[frameA->header.seq][name]  = c2cMSE;
  c2cPSNRMapMap_[frameA->header.seq][name] = c2cPSNR;
  c2pMSEMapMap_[frameA->header.seq][name]  = c2pMSE;
  c2pPSNRMapMap_[frameA->header.seq][name] = c2pPSNR;
  std::cout << __FUNCTION__ << "() "
            << "name=" << name << ", "
            << "frameNumberA=" << frameA->header.seq << ", "
            << "frameNumberB=" << frameB->header.seq << ", "
            << "pointsA=" << frameA->size() << ", "
            << "pointsB=" << frameB->size() << ", "
            << "c2cMSE=" << c2cMSE << ", "
            << "c2cPSNR=" << c2cPSNR << ", "
            << "c2pMSE=" << c2pMSE << ", "
            << "c2pPSNR=" << c2pPSNR << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointA, typename PointB>
void JPCCMetric::addPSNR(const std::string&          name,
                         const GroupOfFrame<PointA>& framesA,
                         const GroupOfFrame<PointB>& framesB,
                         const bool                  parallel) {
  assert(framesA.size() == framesB.size());
  if (!parallel) {
    for (size_t i = 0; i < framesA.size(); i++) { this->addPSNR<PointA, PointB>(name, framesA[i], framesB[i]); }
  } else {
    const auto range = boost::counting_range<size_t>(0, framesA.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->addPSNR<PointA, PointB>(name, framesA[i], framesB[i]);
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointA, typename PointB>
void JPCCMetric::computePSNR(const FramePtr<PointA>& frameA,
                             const FramePtr<PointB>& frameB,
                             double&                 c2cMSE,
                             double&                 c2cPSNR,
                             double&                 c2pMSE,
                             double&                 c2pPSNR) {
  static_assert(pcl::traits::has_normal_v<PointB>, "invalid template type");

  double sseC2c = 0;
  double sseC2p = 0;

  size_t             K = 1;
  std::vector<int>   pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);

  pcl::KdTreeFLANN<PointB> kdtree;

  kdtree.setInputCloud(frameB);

  for (size_t indexA = 0; indexA < frameA->size(); indexA++) {
    const PointA& pointA = (*frameA)[indexA];
    // For point 'i' in A, find its nearest neighbor in B. store it in 'j'
    int ret = kdtree.nearestKSearch(pointA, K, pointIdxKNNSearch, pointKNNSquaredDistance);
    THROW_IF_NOT(ret == K);

    const PointB& pointB = (*frameB)[pointIdxKNNSearch.front()];

    // Compute point-to-point, which should be equal to sqrt( dist[0] )
    const double distProjC2c = pointKNNSquaredDistance.front();

    // Compute point-to-plane, normals in B will be used for point-to-plane
    double distProjC2p;
    if (!std::isnan(pointB.normal_x)) {
      float errX  = pointA.x - pointB.x;
      float errY  = pointA.y - pointB.y;
      float errZ  = pointA.z - pointB.z;
      distProjC2p = errX * pointB.normal_x + errY * pointB.normal_y + errZ * pointB.normal_z;
      distProjC2p *= distProjC2p;
    } else {
      distProjC2p = distProjC2c;
    }

    // mean square distance
    sseC2c += distProjC2c;
    sseC2p += distProjC2p;
  }

  c2cMSE  = sseC2c / frameA->size();
  c2cPSNR = 10 * log10((3 * pow(parameter_.maximumValue, 2)) / c2cMSE);
  c2pMSE  = sseC2p / frameA->size();
  c2pPSNR = 10 * log10((3 * pow(parameter_.maximumValue, 2)) / c2pMSE);
}

}  // namespace jpcc::metric