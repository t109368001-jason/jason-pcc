#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::math {

class Cluster {
 protected:
  double weight_;
  double mean_;
  double variance_;

 public:
  Cluster(const std::vector<Intensity>& samples, double weight, double minimumVariance);

  Cluster(Intensity sample, double weight, double minimumVariance);

  [[nodiscard]] double getProbability(Intensity sample) const;

  [[nodiscard]] double getStaticProbability() const;

  void addSample(Intensity sample, bool matched, double alpha, double minimumVariance);

  void checkVariance(double minimumVariance);

  [[nodiscard]] double getWeight() const;

  [[nodiscard]] double& getWeight();

  [[nodiscard]] double getMean() const;

  [[nodiscard]] double getVariance() const;

  [[nodiscard]] bool operator<(const Cluster& obj) const;
};

}  // namespace jpcc::math