#include <pcl/kdtree/kdtree_flann.h>

namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPoints(const std::string& name, const FramePtr<PointT>& frame) {
  const auto& points = frame->size();
  frameNumberSet_.insert(frame->header.seq);
  pointsNameSet_.insert(name);
  pointsMapMap_[frame->header.seq][name] += points;
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

  double maxC2c = (std::numeric_limits<double>::min)();
  double maxC2p = (std::numeric_limits<double>::min)();
  double sseC2p = 0;
  double sseC2c = 0;

  pcl::KdTreeFLANN<PointB> kdtree;

  kdtree.setInputCloud(frameB);

  for (size_t indexA = 0; indexA < frameA->size(); indexA++) {
    // For point 'i' in A, find its nearest neighbor in B. store it in 'j'
    size_t             K = 1;
    std::vector<int>   pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    int                ret = kdtree.nearestKSearch(frameA->at(indexA), K, pointIdxKNNSearch, pointKNNSquaredDistance);
    THROW_IF_NOT(ret == K);

    size_t indexB = pointIdxKNNSearch.at(0);
    // Compute point-to-point, which should be equal to sqrt( dist[0] )
    double distProjC2c = pointKNNSquaredDistance.at(0);

    // Compute point-to-plane, normals in B will be used for point-to-plane
    std::vector<float> errVector(3);
    for (size_t j = 0; j < 3; j++) { errVector[j] = frameA->at(indexA).data[j] - frameB->at(indexB).data[j]; }
    double distProjC2p = pow(errVector[0] * frameB->at(indexB).data_n[0] + errVector[1] * frameB->at(indexB).data_n[1] +
                                 errVector[2] * frameB->at(indexB).data_n[2],
                             2.F);

    // mean square distance
    sseC2c += distProjC2c;
    if (distProjC2c > maxC2c) { maxC2c = distProjC2c; }
    sseC2p += distProjC2p;
    if (distProjC2p > maxC2p) { maxC2p = distProjC2p; }
  }

  c2cMSE  = float(sseC2c / frameA->size());
  c2cPSNR = 10 * log10((3 * pow(parameter_.maximumValue, 2)) / c2cMSE);
  c2pMSE  = float(sseC2p / frameA->size());
  c2pPSNR = 10 * log10((3 * pow(parameter_.maximumValue, 2)) / c2pMSE);
}

}  // namespace jpcc::metric