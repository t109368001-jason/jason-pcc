#include <jpcc/math/GMM.h>

#include <numeric>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
Cluster::Cluster(const std::vector<float>& samples, const double weight, const double minimumVariance) :
    weight_(weight) {
  assert(!isnan(weight_));
  mean_ = std::accumulate(samples.begin(), samples.end(), float{0});
  mean_ /= static_cast<double>(samples.size());
  variance_ = 0;
  for (auto& sample : samples) { variance_ += pow(sample - mean_, 2); }
  variance_ = variance_ / static_cast<double>(samples.size());
  checkVariance(minimumVariance);
  assert(!isnan(mean_));
  assert(!isnan(variance_));
}
//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getProbability(const float sample) const {
  assert(!isnan(sample));
  double tmp         = exp((-0.5) * pow(sample - mean_, 2) / variance_);
  double probability = tmp / sqrt(2 * M_PI * variance_);
  assert(!isnan(probability));
  return probability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void Cluster::addSample(const float sample, const bool matched, const double alpha, const double minimumVariance) {
  assert(!isnan(sample));
  weight_ = (1 - alpha) * weight_ + (matched ? alpha : 0);
  assert(!isnan(weight_));
  if (matched) {
    double rho = alpha * getProbability(sample);
    mean_      = (1 - rho) * mean_ + rho * sample;
    variance_  = (1 - rho) * variance_ + rho * pow(sample - mean_, 2);
    checkVariance(minimumVariance);
    assert(!isnan(mean_));
    assert(!isnan(variance_));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void Cluster::checkVariance(const double minimumVariance) {
  if (variance_ < minimumVariance) { variance_ = minimumVariance; }
}

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getWeight() const { return weight_; }

//////////////////////////////////////////////////////////////////////////////////////////////
double& Cluster::getWeight() { return weight_; }

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getMean() const { return mean_; }

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getVariance() const { return variance_; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool Cluster::operator<(const Cluster& obj) const { return mean_ < obj.mean_; }

}  // namespace jpcc::math