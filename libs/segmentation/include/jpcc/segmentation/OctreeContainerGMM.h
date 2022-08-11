#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/math/GMM.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>

namespace jpcc::segmentation {

template <typename PointT>
class OctreeContainerGMM : virtual public octree::OctreeContainerLastPoint<PointT>, public math::GMM {
 protected:
  shared_ptr<std::vector<float>> trainSamples_;

 public:
  OctreeContainerGMM();

  void reset() override;

  void addPoint(const PointT& point) override;

  void addTrainSample();

  virtual void build(
      int nTrain, int K, double alpha, double minimumVariance, const std::vector<float>& alternateCentroids);

  [[nodiscard]] bool isStatic(std::vector<double> staticThresholdVector, std::vector<double> nullStaticThresholdVector);

  virtual void updateModel(double alpha, double nullAlpha, double minimumVariance);
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/OctreeContainerGMM.hpp>
