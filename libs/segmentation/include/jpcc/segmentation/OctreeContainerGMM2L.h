#pragma once

#include <array>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/math/GMM.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>
#include <jpcc/segmentation/IOctreeContainerGMM.h>

namespace jpcc::segmentation {

template <typename PointT>
class OctreeContainerGMM2L : public IOctreeContainerGMM, virtual public octree::OctreeContainerLastPoint<PointT> {
 public:
  static constexpr int SIZE        = 2;
  static constexpr int SHORT_INDEX = 0;
  static constexpr int LONG_INDEX  = 1;

 protected:
  std::array<shared_ptr<std::vector<float>>, SIZE> trainSamplesVector_;
  std::array<math::GMM, SIZE>                      gmms_;

 public:
  OctreeContainerGMM2L();

  void reset() override;

  void addPoint(const PointT& point) override;

  [[nodiscard]] bool isBuilt(int index) const override;

  void addTrainSample() override;

  void build(int                       index,
             int                       nTrain,
             int                       K,
             double                    alpha,
             double                    minimumVariance,
             const std::vector<float>& alternateCentroids) override;

  [[nodiscard]] bool isStatic(const std::vector<double>& staticThresholdVector,
                              const std::vector<double>& nullStaticThresholdVector,
                              const std::vector<bool>&   outputExistsPointOnlyVector) override;

  void updateModel(int index, double alpha, double nullAlpha, double minimumVariance) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/OctreeContainerGMM2L.hpp>
