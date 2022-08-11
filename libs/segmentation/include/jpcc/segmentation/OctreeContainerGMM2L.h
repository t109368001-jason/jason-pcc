#pragma once

#include <array>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/math/GMM.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>

namespace jpcc::segmentation {

template <typename PointT>
class OctreeContainerGMM2L : virtual public octree::OctreeContainerLastPoint<PointT> {
 public:
  static constexpr int SIZE        = 2;
  static constexpr int SHORT_INDEX = 0;
  static constexpr int LONG_INDEX  = 0;

 protected:
  std::array<shared_ptr<std::vector<float>>, SIZE> trainSamplesVector_;
  std::array<math::GMM, SIZE>                      gmms_;

 public:
  OctreeContainerGMM2L();

  void reset() override;

  void addPoint(const PointT& point) override;

  [[nodiscard]] bool isBuilt(int index) const;

  void addTrainSample();

  virtual void build(
      int index, int nTrain, int K, double alpha, double minimumVariance, const std::vector<float>& alternateCentroids);

  [[nodiscard]] bool isStatic(std::vector<double> dynamicThresholdLEVector,
                              std::vector<double> staticThresholdGTVector);

  virtual void updateModel(int index, double alpha, double nullAlpha, double minimumVariance);
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/OctreeContainerGMM2L.hpp>
