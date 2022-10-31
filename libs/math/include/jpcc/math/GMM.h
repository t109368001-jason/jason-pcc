#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/math/Cluster.h>

namespace jpcc::math {

class GMM {
 public:
  using Ptr = shared_ptr<GMM>;

 protected:
  std::vector<Cluster> clusters_;

 public:
  GMM() = default;

  void buildN(std::vector<int32_t>&     samples,
              int                       K,
              int                       N,
              double                    minimumVariance,
              int32_t                   nullSample,
              const std::vector<float>& alternateCentroids);

  [[nodiscard]] bool isBuilt() const;

  [[nodiscard]] double getProbability(int32_t sample);

  [[nodiscard]] double getStaticProbability();

  [[nodiscard]] size_t getOptimalModelIndex(int32_t sample) const;

  void updateModel(int32_t sample, double alpha, double minimumVariance);

  void normalizeWeights();

  [[nodiscard]] const std::vector<Cluster>& getClusters() const;

  void reset();
};

}  // namespace jpcc::math