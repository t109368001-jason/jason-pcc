#include <pcl/kdtree/kdtree_flann.h>

namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPoints(const std::string& name, const FramePtr<PointT>& frame) {
  const auto& points = frame->size();
  frameNumberSet_.insert(frame->header.seq);
  pointsNameSet_.insert(name);
  pointsMapMap_[frame->header.seq][name] += points;
  std::cout << __FUNCTION__ << "() "
            << "name=" << name << ", "
            << "frameNumber=" << frame->header.seq << ", "
            << "points=" << points << std::endl;
  addBytes(name, frame->header.seq, points * sizeof(float) * 3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPoints(const std::string& name, const GroupOfFrame<PointT>& frames) {
  for (const auto& frame : frames) { addPoints<PointT>(name, frame); }
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
            << "c2cMSE=" << c2cMSE << ", "
            << "c2cPSNR=" << c2cPSNR << ", "
            << "c2pMSE=" << c2pMSE << ", "
            << "c2pPSNR=" << c2pPSNR << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointA, typename PointB>
void JPCCMetric::addPSNR(const std::string&          name,
                         const GroupOfFrame<PointA>& framesA,
                         const GroupOfFrame<PointB>& framesB) {
  assert(framesA.size() == framesB.size());
  for (size_t i = 0; i < framesA.size(); i++) { addPSNR(name, framesA.at(i), framesB.at(i)); }
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
    const PointA& pointA = frameA->at(indexA);
    // For point 'i' in A, find its nearest neighbor in B. store it in 'j'
    int ret = kdtree.nearestKSearch(pointA, K, pointIdxKNNSearch, pointKNNSquaredDistance);
    THROW_IF_NOT(ret == K);

    const PointB& pointB = frameB->at(pointIdxKNNSearch.at(0));

    // Compute point-to-point, which should be equal to sqrt( dist[0] )
    const double distProjC2c = pointKNNSquaredDistance.at(0);

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