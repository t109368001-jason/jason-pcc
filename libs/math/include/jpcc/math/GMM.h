#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::math {

class Cluster {
 public:
  using Ptr = shared_ptr<Cluster>;

  using SampleT = float;

 protected:
  double weight_;
  double mean_;
  double variance_;
  double alpha_;  // weight learning rate

 public:
  Cluster(const std::vector<SampleT>& samples, double weight, double alpha);

  [[nodiscard]] double getProbability(SampleT sample) const;

  void addSample(SampleT sample, bool matched);

  [[nodiscard]] double getWeight() const;

  [[nodiscard]] double& getWeight();

  void setWeight(double weight);

  [[nodiscard]] double getMean() const;

  [[nodiscard]] double getVariance() const;
};

class GMM {
 public:
  using Ptr = shared_ptr<GMM>;

  using SampleT = Cluster::SampleT;

 protected:
  const int                 K_;
  std::vector<Cluster::Ptr> clusters_;

 public:
  GMM(const std::vector<SampleT>& samples, int K, double alpha = 0.05);

  [[nodiscard]] double getProbability(SampleT sample);

  [[nodiscard]] double getStaticProbability();

  [[nodiscard]] std::pair<size_t, double> getOptimalModelIndex(SampleT sample) const;

  [[nodiscard]] std::pair<size_t, double> getOptimalModelIndex(const std::vector<SampleT>& samples) const;

  void updateModel(SampleT sample, bool normalizeWeights = true);

  void updateModel(const std::vector<SampleT>& samples, bool normalizeWeights = true);

  void normalizeWeights();

  [[nodiscard]] const std::vector<Cluster::Ptr>& getClusters() const;
};

}  // namespace jpcc::math