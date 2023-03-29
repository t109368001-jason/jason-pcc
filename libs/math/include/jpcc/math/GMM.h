#pragma once

#include <set>

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

  void buildN(std::vector<Intensity>&    samples,
              int                        K,
              int                        N,
              double                     minimumVariance,
              Intensity                  nullSample,
              const std::set<Intensity>& alternateCentroids);

  [[nodiscard]] bool isBuilt() const;

  [[nodiscard]] double getProbability(Intensity sample);

  [[nodiscard]] size_t getOptimalModelIndex(Intensity sample) const;

  void updateModel(Intensity sample, double alpha, double minimumVariance);

  void normalizeWeights();

  [[nodiscard]] const std::vector<Cluster>& getClusters() const;

  void reset();
};

}  // namespace jpcc::math