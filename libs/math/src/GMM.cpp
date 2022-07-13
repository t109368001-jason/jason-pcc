#include <jpcc/math/GMM.h>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
Cluster::Cluster(const std::vector<SampleT>& samples,
                 const double                weight,
                 const double                alpha,
                 const double                minimumVariance) :
    weight_(weight), alpha_(alpha), minimumVariance_(minimumVariance) {
  assert(!isnan(weight_));
  assert(!isnan(alpha_));
  assert(!isnan(minimumVariance));
  mean_ = 0;
  for (auto& sample : samples) { mean_ += sample; }
  mean_ /= static_cast<double>(samples.size());
  variance_ = 0;
  for (auto& sample : samples) { variance_ += pow(sample - mean_, 2); }
  variance_ = sqrt(variance_ / static_cast<double>(samples.size()));
  checkVariance();
  assert(!isnan(mean_));
  assert(!isnan(variance_));
}
//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getProbability(const SampleT sample) const {
  assert(!isnan(sample));
  double tmp         = exp((-0.5) * pow(sample - mean_, 2) / variance_);
  double probability = tmp / sqrt(2 * M_PI * variance_);
  assert(!isnan(probability));
  return probability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void Cluster::addSample(const SampleT sample, const bool matched) {
  assert(!isnan(sample));
  weight_    = (1 - alpha_) * weight_ + (matched ? alpha_ : 0);
  double rho = alpha_ * getProbability(sample);
  mean_      = (1 - rho) * mean_ + rho * sample;
  variance_  = (1 - rho) * variance_ + rho * pow(sample - mean_, 2);
  checkVariance();
  assert(!isnan(weight_));
  assert(!isnan(mean_));
  assert(!isnan(variance_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void Cluster::checkVariance() {
  if (variance_ < minimumVariance_) { variance_ = minimumVariance_; }
}

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getWeight() const { return weight_; }

//////////////////////////////////////////////////////////////////////////////////////////////
double& Cluster::getWeight() { return weight_; }

//////////////////////////////////////////////////////////////////////////////////////////////
void Cluster::setWeight(const double weight) {
  assert(!isnan(weight));
  weight_ = weight;
}

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getMean() const { return mean_; }

//////////////////////////////////////////////////////////////////////////////////////////////
double Cluster::getVariance() const { return variance_; }

//////////////////////////////////////////////////////////////////////////////////////////////
GMM::GMM(vector<SampleT>&      samples,
         const int             K,
         const double          alpha,
         const double          minimumVariance,
         std::vector<SampleT>& alternateCentroids) :
    K_(K) {
  clusters_.resize(K_);
  size_t k;

  vector<SampleT>         centroids;
  vector<vector<SampleT>> clusters(K_);
  // K-Means
  // get centroids
  std::vector<SampleT> uniqueSamples;
  for (const auto& sample : samples) {
    bool anyEqual = false;
    for (const auto& uniqueSample : uniqueSamples) {
      if (sample == uniqueSample) {
        anyEqual = true;
        break;
      }
    }
    if (!anyEqual) {
      uniqueSamples.push_back(sample);
      if (uniqueSamples.size() >= K_) { break; }
    }
  }
  if (uniqueSamples.size() == K_) {
    while (centroids.size() < K_) {
      SampleT sample = samples.at(rand() % samples.size());
      bool    exists = false;
      for (float centroid : centroids) {
        if (sample == centroid) {
          exists = true;
          break;
        }
      }
      if (!exists) { centroids.push_back(sample); }
    }
  } else {
    for (const auto& uniqueSample : uniqueSamples) { centroids.push_back(uniqueSample); }
    k = 0;
    while (centroids.size() < K_) {
      centroids.push_back(alternateCentroids.at(k));
      samples.push_back(alternateCentroids.at(k));
      k++;
    }
  }

  bool isConverged = false;
  while (!isConverged) {
    for (k = 0; k < K_; k++) { clusters.at(k).clear(); }
    std::vector<SampleT> previousCentroids(K_);
    copy(centroids.begin(), centroids.end(), previousCentroids.begin());

    for (const auto& sample : samples) {
      size_t  minIndex    = 0;
      SampleT minDistance = abs(sample - centroids.at(0));
      for (k = 1; k < K_; k++) {
        SampleT distance = abs(sample - centroids.at(k));
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
      centroids.at(k) = static_cast<SampleT>(sum / static_cast<double>(clusters.at(k).size()));
    }

    isConverged = true;
    for (k = 0; k < K_; k++) {
      if (centroids.at(k) != previousCentroids.at(k)) { isConverged = false; }
      previousCentroids.at(k) = centroids.at(k);
    }
  }

  for (k = 0; k < K_; k++) {
    double weight   = static_cast<double>(clusters.at(k).size()) / static_cast<double>(samples.size());
    clusters_.at(k) = jpcc::make_shared<Cluster>(clusters.at(k), weight, alpha, minimumVariance);
  }
  sort(clusters_.begin(), clusters_.end(), [](const Cluster::Ptr& cluster1, const Cluster::Ptr& cluster2) {
    return cluster1->getMean() < cluster2->getMean();
  });
}

//////////////////////////////////////////////////////////////////////////////////////////////
[[nodiscard]] double GMM::getProbability(SampleT sample) {
  assert(!isnan(sample));
  double totalProbability = 0.0;

  for (const auto& cluster : clusters_) {
    double probability = cluster->getWeight() * cluster->getProbability(sample);
    if (cluster->getMean() < 0) {
      totalProbability -= probability;
    } else {
      totalProbability += probability;
    }
  }
  return totalProbability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
[[nodiscard]] double GMM::getStaticProbability() {
  double totalProbability = 0.0;

  for (const auto& cluster : clusters_) {
    double probability = cluster->getWeight() * cluster->getProbability(static_cast<SampleT>(cluster->getMean()));
    if (cluster->getMean() < 0) {
      totalProbability -= probability;
    } else {
      totalProbability += probability;
    }
  }
  return totalProbability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pair<size_t, double> GMM::getOptimalModelIndex(const SampleT sample) const {
  assert(!isnan(sample));
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
pair<size_t, double> GMM::getOptimalModelIndex(const vector<SampleT>& samples) const {
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
void GMM::updateModel(SampleT sample, bool normalizeWeights) {
  assert(!isnan(sample));
  size_t optimalIndex = getOptimalModelIndex(sample).first;
  for (size_t k = 0; k < K_; k++) { clusters_.at(k)->addSample(sample, k == optimalIndex); }
  if (normalizeWeights) { this->normalizeWeights(); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::updateModel(const vector<SampleT>& samples, bool normalizeWeights) {
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