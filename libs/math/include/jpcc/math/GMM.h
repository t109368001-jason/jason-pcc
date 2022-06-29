#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::math {

class Cluster {
 public:
  using Ptr = shared_ptr<Cluster>;

 protected:
  double weight_;
  double mean_;
  double variance_;
  double alpha_;  // weight learning rate

 public:
  Cluster(const std::vector<double>& samples, double weight, double alpha);

  [[nodiscard]] double getProbability(double sample) const;

  void addSample(double sample, bool matched);

  double getWeight() const;

  double& getWeight();

  void setWeight(double weight);

  double getMean() const;

  double getVariance() const;
};

class GMM {
 public:
  using Ptr = shared_ptr<GMM>;

 protected:
  const int                 K_;
  std::vector<Cluster::Ptr> clusters_;

 public:
  GMM(const std::vector<double>& samples, int K, double alpha = 0.05);

  [[nodiscard]] std::pair<size_t, double> getOptimalModelIndex(double sample) const;

  [[nodiscard]] std::pair<size_t, double> getOptimalModelIndex(const std::vector<double>& samples) const;

  void updateModel(double sample, bool normalizeWeights = true);

  void updateModel(const std::vector<double>& samples, bool normalizeWeights = true);

  void normalizeWeights();

  [[nodiscard]] const std::vector<Cluster::Ptr>& getClusters() const;
};

}  // namespace jpcc::math