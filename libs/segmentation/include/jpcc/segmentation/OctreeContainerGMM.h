#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/math/GMM.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>

namespace jpcc::segmentation {

class OctreeContainerGMM : virtual public octree::OctreeContainerLastPoint {
 protected:
  shared_ptr<std::vector<float>> trainSamples_;
  math::GMM::Ptr                 gmm_;

 public:
  OctreeContainerGMM();

  void reset() override;

  void addPoint(const PointXYZINormal& point) override;

  void addTrainSample();

  virtual void build(
      int nTrain, int K, double alpha, double minimumVariance, const std::vector<float>& alternateCentroids);

  virtual void updateModel(double alpha, double minimumVariance);

  [[nodiscard]] virtual bool isBuilt() const;

  [[nodiscard]] float getIntensityNormalized();

  [[nodiscard]] math::GMM::Ptr& getGMM();
};

}  // namespace jpcc::segmentation