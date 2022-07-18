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

  virtual void addPoint(const PointXYZINormal& point);

  void addTrainSample();

  virtual void build(int nTrain, int K, double alpha, double minimumVariance, std::vector<float>& alternateCentroids);

  virtual bool isBuilt() const;

  [[nodiscard]] float getIntensity() const;

  [[nodiscard]] float& getIntensity();

  [[nodiscard]] const shared_ptr<std::vector<float>>& getTrainSamples() const;

  [[nodiscard]] shared_ptr<std::vector<float>>& getTrainSamples();

  [[nodiscard]] const math::GMM::Ptr& getGMM() const;

  [[nodiscard]] math::GMM::Ptr& getGMM();
};

}  // namespace jpcc::octree