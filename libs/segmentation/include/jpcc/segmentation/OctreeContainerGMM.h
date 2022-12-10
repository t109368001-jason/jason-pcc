#pragma once

#include <set>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/common/IJPCCSegmentationContext.h>
#include <jpcc/math/GMM.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>
#include <jpcc/segmentation/IOctreeContainerGMM.h>

namespace jpcc::segmentation {

class OctreeContainerGMM : public IOctreeContainerGMM,
                           virtual public octree::OctreeContainerLastPoint<PointSegmentation>,
                           public math::GMM {
 public:
  static constexpr int SIZE = 1;

 protected:
  shared_ptr<std::vector<Intensity>> trainSamples_;

 public:
  OctreeContainerGMM();

  void reset() override;

  void addPoint(const PointSegmentation& point) override;

  [[nodiscard]] bool isBuilt(int index) const override;

  void addTrainSample() override;

  void build(int                        index,
             int                        nTrain,
             int                        K,
             double                     alpha,
             double                     minimumVariance,
             const std::set<Intensity>& alternateCentroids) override;

  [[nodiscard]] bool isStatic(const std::vector<double>& staticThreshold1Vector,
                              const std::vector<double>& staticThreshold2Vector,
                              const std::vector<double>& nullStaticThreshold1Vector,
                              const std::vector<double>& nullStaticThreshold2Vector,
                              const std::vector<bool>&   outputExistsPointOnlyVector,
                              bool                       lastIsStatic) override;

  void updateModel(int index, double alpha, double nullAlpha, double minimumVariance) override;
};

}  // namespace jpcc::segmentation
