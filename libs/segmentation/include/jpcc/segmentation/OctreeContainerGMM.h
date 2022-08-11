#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/math/GMM.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>
#include <jpcc/segmentation/IOctreeContainerGMM.h>

namespace jpcc::segmentation {

template <typename PointT>
class OctreeContainerGMM : public IOctreeContainerGMM,
                           virtual public octree::OctreeContainerLastPoint<PointT>,
                           public math::GMM {
 public:
  static constexpr int SIZE = 1;

 protected:
  shared_ptr<std::vector<float>> trainSamples_;

 public:
  OctreeContainerGMM();

  void reset() override;

  void addPoint(const PointT& point) override;

  [[nodiscard]] bool isBuilt(int index) const override;

  void addTrainSample() override;

  virtual void build(int                       index,
                     int                       nTrain,
                     int                       K,
                     double                    alpha,
                     double                    minimumVariance,
                     const std::vector<float>& alternateCentroids) override;

  [[nodiscard]] bool isStatic(std::vector<double> staticThresholdVector,
                              std::vector<double> nullStaticThresholdVector) override;

  virtual void updateModel(int index, double alpha, double nullAlpha, double minimumVariance) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/OctreeContainerGMM.hpp>
