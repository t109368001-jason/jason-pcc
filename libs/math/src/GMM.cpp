#include <jpcc/math/GMM.h>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
Cluster::Cluster(const std::vector<double>& samples, const double weight, const double alpha) :
    weight_(weight), alpha_(alpha) {
  mean_ = 0;
  for (auto& sample : samples) { mean_ += sample; }
  mean_ /= samples.size();
  variance_ = 0;
  for (auto& sample : samples) { variance_ += pow(sample - mean_, 2); }
  variance_ = sqrt(variance_ / samples.size());
}

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getProbability(const double sample) const {
  double tmp = exp((-0.5) * pow(sample - mean_, 2) / pow(variance_, 2));
  return tmp / sqrt(2 * M_PI * pow(variance_, 2));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void Cluster::addSample(const double sample, const bool matched) {
  weight_    = (1 - alpha_) * weight_ + (matched ? alpha_ : 0);
  double rho = alpha_ * getProbability(sample);
  mean_      = (1 - rho) * mean_ + rho * sample;
  variance_  = sqrt((1 - rho) * pow(variance_, 2) + rho * pow(sample - mean_, 2));
}

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getWeight() const { return weight_; }

//////////////////////////////////////////////////////////////////////////////////////////////
double& Cluster::getWeight() { return weight_; }

//////////////////////////////////////////////////////////////////////////////////////////////
void Cluster::setWeight(const double weight) { weight_ = weight; }

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getMean() const { return mean_; }

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getVariance() const { return variance_; }

//////////////////////////////////////////////////////////////////////////////////////////////
GMM::GMM(const std::vector<double>& samples, const int K, const double alpha) : K_(K) {
  clusters_.resize(K_);
  size_t k;

  vector<double>         centroids(K_);
  vector<vector<double>> clusters(K_);
  // K-Means
  // get centroids
  for (k = 0; k < K_; k++) {
    double sample = samples.at(rand() % samples.size());
    bool   same   = false;
    for (size_t i = 0; i < k; i++) {
      if (sample == centroids.at(i)) {
        same = true;
        break;
      }
    }
    if (same) {
      k--;
    } else {
      centroids.at(k) = sample;
    }
  }

  bool isConverged = false;
  while (!isConverged) {
    for (k = 0; k < K_; k++) { clusters.at(k).clear(); }
    std::vector<double> previousCentroids(K_);
    copy(centroids.begin(), centroids.end(), previousCentroids.begin());

    for (const auto& sample : samples) {
      size_t minIndex    = 0;
      double minDistance = abs(sample - centroids.at(0));
      for (k = 1; k < K_; k++) {
        double distance = abs(sample - centroids.at(k));
        if (distance < minDistance) {
          minIndex    = k;
          minDistance = distance;
        }
      }
      clusters.at(minIndex).push_back(sample);
    }
    for (k = 0; k < K_; k++) {
      double sum = 0.0;
      for (const auto& sample : clusters.at(k)) { sum += sample; }
      centroids.at(k) = sum / clusters.at(k).size();
    }

    isConverged = true;
    for (k = 0; k < K_; k++) {
      if (centroids.at(k) != previousCentroids.at(k)) { isConverged = false; }
      previousCentroids.at(k) = centroids.at(k);
    }
  }

  for (k = 0; k < K_; k++) { clusters_.at(k) = jpcc::make_shared<Cluster>(clusters.at(k), 1.0 / K_, alpha); }
  sort(clusters_.begin(), clusters_.end(), [](const Cluster::Ptr& cluster1, const Cluster::Ptr& cluster2) {
    return cluster1->getMean() < cluster2->getMean();
  });
}

//////////////////////////////////////////////////////////////////////////////////////////////
pair<size_t, double> GMM::getOptimalModelIndex(const double sample) const {
  std::pair<size_t, double> optimalIndexProbability;
  optimalIndexProbability.first  = 0;
  optimalIndexProbability.second = clusters_.at(0)->getProbability(sample);
  for (size_t k = 1; k < K_; k++) {
    double probability = clusters_.at(k)->getProbability(sample);
    if (probability > optimalIndexProbability.second) {
      optimalIndexProbability.first  = k;
      optimalIndexProbability.second = probability;
    }
  }
  return optimalIndexProbability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pair<size_t, double> GMM::getOptimalModelIndex(const vector<double>& samples) const {
  std::pair<size_t, double> optimalIndexProbability = getOptimalModelIndex(samples.at(0));
  for (size_t i = 1; i < samples.size(); i++) {
    std::pair<size_t, double> _optimalIndexProbability = getOptimalModelIndex(samples.at(i));
    if (_optimalIndexProbability.second > optimalIndexProbability.second) {
      optimalIndexProbability = _optimalIndexProbability;
    }
  }
  return optimalIndexProbability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::updateModel(double sample, bool normalizeWeights) {
  size_t optimalIndex = getOptimalModelIndex(sample).first;
  for (size_t k = 0; k < K_; k++) { clusters_.at(k)->addSample(sample, k == optimalIndex); }
  if (normalizeWeights) { this->normalizeWeights(); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::updateModel(const vector<double>& samples, bool normalizeWeights) {
  for (const auto& sample : samples) { updateModel(sample, false); }
  if (normalizeWeights) { this->normalizeWeights(); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::normalizeWeights() {
  double sum = 0.0;
  for (const Cluster::Ptr& cluster : clusters_) { sum += cluster->getWeight(); }
  double multiplier = 1.0 / sum;

  for (Cluster::Ptr& cluster : clusters_) { cluster->getWeight() *= multiplier; }
}

//////////////////////////////////////////////////////////////////////////////////////////////
const std::vector<Cluster::Ptr>& GMM::getClusters() const { return clusters_; }

}  // namespace jpcc::math