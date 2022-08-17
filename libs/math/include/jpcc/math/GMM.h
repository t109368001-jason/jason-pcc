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

  void buildN(std::vector<float>&       samples,
              int                       K,
              int                       N,
              double                    minimumVariance,
              float                     nullSample,
              const std::vector<float>& alternateCentroids);

  [[nodiscard]] bool isBuilt() const;

  [[nodiscard]] double getProbability(float sample);

  [[nodiscard]] double getStaticProbability();

  [[nodiscard]] size_t getOptimalModelIndex(float sample) const;

  void updateModel(float sample, double alpha, double minimumVariance);

  void normalizeWeights();

  [[nodiscard]] const std::vector<Cluster>& getClusters() const;

  void reset();
};

}  // namespace jpcc::math