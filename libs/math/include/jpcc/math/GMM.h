#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::math {

class Cluster {
 public:
  using SampleT = float;

 protected:
  double weight_;
  double mean_;
  double variance_;

 public:
  Cluster(const std::vector<SampleT>& samples, double weight, double minimumVariance);

  [[nodiscard]] double getProbability(SampleT sample) const;

  void addSample(SampleT sample, bool matched, double alpha, double minimumVariance);

  void checkVariance(double minimumVariance);

  [[nodiscard]] double getWeight() const;

  [[nodiscard]] double& getWeight();

  [[nodiscard]] double getMean() const;

  [[nodiscard]] double getVariance() const;
};

class GMM {
 public:
  using Ptr = shared_ptr<GMM>;

  using SampleT = Cluster::SampleT;

 protected:
  std::vector<Cluster> clusters_;

 public:
  GMM() = default;

  GMM(std::vector<SampleT>& samples, int K, double minimumVariance, const std::vector<SampleT>& alternateCentroids);

  void build(std::vector<SampleT>&       samples,
             int                         K,
             double                      minimumVariance,
             const std::vector<SampleT>& alternateCentroids);

  [[nodiscard]] bool isBuilt();

  [[nodiscard]] double getProbability(SampleT sample);

  [[nodiscard]] double getStaticProbability();

  [[nodiscard]] std::pair<size_t, double> getOptimalModelIndex(SampleT sample) const;

  void updateModel(SampleT sample, double alpha, double minimumVariance);

  void normalizeWeights();

  [[nodiscard]] const std::vector<Cluster>& getClusters() const;

  void reset();
};

}  // namespace jpcc::math