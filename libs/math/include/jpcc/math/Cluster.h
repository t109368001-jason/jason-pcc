#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::math {

class Cluster {
 protected:
  double weight_;
  double mean_;
  double variance_;

 public:
  Cluster(const std::vector<int32_t>& samples, double weight, double minimumVariance);

  Cluster(int32_t sample, double weight, double minimumVariance);

  [[nodiscard]] double getProbability(int32_t sample) const;

  [[nodiscard]] double getStaticProbability() const;

  void addSample(int32_t sample, bool matched, double alpha, double minimumVariance);

  void checkVariance(double minimumVariance);

  [[nodiscard]] double getWeight() const;

  [[nodiscard]] double& getWeight();

  [[nodiscard]] double getMean() const;

  [[nodiscard]] double getVariance() const;

  [[nodiscard]] bool operator<(const Cluster& obj) const;
};

}  // namespace jpcc::math