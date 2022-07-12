#pragma once

#include <vector>

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>
#include <jpcc/math/GMM.h>

namespace jpcc::octree {

class OctreeContainerGMM : virtual public pcl::octree::OctreeContainerBase {
 protected:
  shared_ptr<std::vector<float>> trainSamples_;
  float                          intensity_;
  math::GMM::Ptr                 gmm_;

 public:
  OctreeContainerGMM();

  void reset() override;

  virtual void addPoint(const PointXYZINormal& point);

  void initGMM(int K, double alpha, double minimumVariance, std::vector<float>& alternateCentroids);

  [[nodiscard]] float getIntensity() const;

  [[nodiscard]] float& getIntensity();

  void setIntensity(float intensity);

  [[nodiscard]] const shared_ptr<std::vector<float>>& getTrainSamples() const;

  [[nodiscard]] shared_ptr<std::vector<float>>& getTrainSamples();

  [[nodiscard]] const math::GMM::Ptr& getGMM() const;

  [[nodiscard]] math::GMM::Ptr& getGMM();
};

}  // namespace jpcc::octree