#pragma once

#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/math/GMM.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>

namespace jpcc::segmentation {

class IOctreeContainerGMM {
 public:
  [[nodiscard]] virtual bool isBuilt(int index) const = 0;

  virtual void addTrainSample() = 0;

  virtual void build(int                       index,
                     int                       nTrain,
                     int                       K,
                     double                    alpha,
                     double                    minimumVariance,
                     const std::vector<float>& alternateCentroids) = 0;

  [[nodiscard]] virtual bool isStatic(const std::vector<double>& staticThresholdVector,
                                      const std::vector<double>& nullStaticThresholdVector) = 0;

  virtual void updateModel(int index, double alpha, double nullAlpha, double minimumVariance) = 0;
};

}  // namespace jpcc::segmentation
