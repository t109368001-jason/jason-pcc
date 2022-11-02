#pragma once

#include <set>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/math/GMM.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>
#include <jpcc/segmentation/IOctreeContainerGMM.h>

namespace jpcc::segmentation {

class OctreeContainerGMM : public IOctreeContainerGMM,
                           virtual public octree::OctreeContainerLastPoint<pcl::PointXYZI>,
                           public math::GMM {
 public:
  static constexpr int SIZE = 1;

 protected:
  shared_ptr<std::vector<Intensity>> trainSamples_;

 public:
  OctreeContainerGMM();

  void reset() override;

  void addPoint(const pcl::PointXYZI& point) override;

  [[nodiscard]] bool isBuilt(int index) const override;

  void addTrainSample() override;

  void build(int                        index,
             int                        nTrain,
             int                        K,
             double                     alpha,
             double                     minimumVariance,
             const std::set<Intensity>& alternateCentroids) override;

  [[nodiscard]] bool isStatic(const std::vector<double>& staticThresholdVector,
                              const std::vector<double>& nullStaticThresholdVector,
                              const std::vector<bool>&   outputExistsPointOnlyVector) override;

  void updateModel(int index, double alpha, double nullAlpha, double minimumVariance) override;
};

}  // namespace jpcc::segmentation
