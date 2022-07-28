#include <jpcc/math/GMM.h>

#include <numeric>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
Cluster::Cluster(const std::vector<SampleT>& samples, const double weight, const double minimumVariance) :
    weight_(weight) {
  assert(!isnan(weight_));
  mean_ = std::accumulate(samples.begin(), samples.end(), SampleT{0});
  mean_ /= static_cast<double>(samples.size());
  variance_ = 0;
  for (auto& sample : samples) { variance_ += pow(sample - mean_, 2); }
  variance_ = variance_ / static_cast<double>(samples.size());
  checkVariance(minimumVariance);
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
void Cluster::addSample(const SampleT sample, const bool matched, const double alpha, const double minimumVariance) {
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
GMM::GMM(vector<SampleT>&            samples,
         const int                   K,
         const double                minimumVariance,
         const std::vector<SampleT>& seedCentroids,
         const std::vector<SampleT>& alternateCentroids) {
  build(samples, K, minimumVariance, seedCentroids, alternateCentroids);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::build(vector<SampleT>&            samples,
                const int                   K,
                const double                minimumVariance,
                const std::vector<SampleT>& seedCentroids,
                const std::vector<SampleT>& alternateCentroids) {
  size_t k;

  vector<SampleT> centroids;
  for (const auto& seedCentroid : seedCentroids) {
    centroids.push_back(seedCentroid);
    samples.push_back(seedCentroid);
  }
  vector<vector<SampleT>> clusters(K);
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
      if (uniqueSamples.size() >= K) { break; }
    }
  }
  if (uniqueSamples.size() >= K) {
    while (centroids.size() < K) {
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
    for (size_t i = 0; i < uniqueSamples.size() && centroids.size() < K; i++) {
      SampleT sample = uniqueSamples.at(i);
      bool    exists = false;
      for (float centroid : centroids) {
        if (sample == centroid) {
          exists = true;
          break;
        }
      }
      if (!exists) { centroids.push_back(sample); }
    }
    for (size_t i = 0; i < alternateCentroids.size() && centroids.size() < K; i++) {
      centroids.push_back(alternateCentroids.at(i));
      samples.push_back(alternateCentroids.at(i));
    }
  }

  std::vector<SampleT> previousCentroids;
  for (const auto& centroid : centroids) {
    assert(!isnan(centroid));
    previousCentroids.push_back(centroid);
  }

  bool isConverged = false;
  while (!isConverged) {
    for (k = 0; k < K; k++) { clusters.at(k).clear(); }

    for (const auto& sample : samples) {
      size_t  minIndex    = 0;
      SampleT minDistance = abs(sample - centroids.at(0));
      for (k = 1; k < K; k++) {
        SampleT distance = abs(sample - centroids.at(k));
        if (distance < minDistance) {
          minIndex    = k;
          minDistance = distance;
        }
      }
      clusters.at(minIndex).push_back(sample);
    }

    for (k = 0; k < K; k++) {
      assert(!clusters.at(k).empty());
      const SampleT sum = accumulate(clusters.at(k).begin(), clusters.at(k).end(), SampleT{0});
      centroids.at(k)   = static_cast<SampleT>(sum / static_cast<double>(clusters.at(k).size()));
    }

    isConverged = true;
    for (k = 0; k < K; k++) {
      if (centroids.at(k) != previousCentroids.at(k)) { isConverged = false; }
    }
    copy(centroids.begin(), centroids.end(), previousCentroids.begin());
  }

  for (k = 0; k < K; k++) {
    double weight = static_cast<double>(clusters.at(k).size()) / static_cast<double>(samples.size());
    clusters_.emplace_back(clusters.at(k), weight, minimumVariance);
  }
  sort(clusters_.begin(), clusters_.end(),
       [](const Cluster& cluster1, const Cluster& cluster2) { return cluster1.getMean() < cluster2.getMean(); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool GMM::isBuilt() { return !clusters_.empty(); }

//////////////////////////////////////////////////////////////////////////////////////////////
[[nodiscard]] double GMM::getProbability(SampleT sample) {
  assert(!isnan(sample));
  double totalProbability = 0.0;

  for (const auto& cluster : clusters_) {
    double probability = cluster.getWeight() * cluster.getProbability(sample);
    if (cluster.getMean() < 0) {
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
    double probability = cluster.getWeight() * cluster.getProbability(static_cast<SampleT>(cluster.getMean()));
    if (cluster.getMean() < 0) {
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
  optimalIndexProbability.second = clusters_.at(0).getWeight() * clusters_.at(0).getProbability(sample);
  for (size_t k = 1; k < clusters_.size(); k++) {
    double probability = clusters_.at(k).getWeight() * clusters_.at(k).getProbability(sample);
    if (probability > optimalIndexProbability.second) {
      optimalIndexProbability.first  = k;
      optimalIndexProbability.second = probability;
    }
  }
  return optimalIndexProbability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::updateModel(SampleT sample, const double alpha, const double minimumVariance) {
  assert(!isnan(sample));
  size_t optimalIndex = getOptimalModelIndex(sample).first;
  for (size_t k = 0; k < clusters_.size(); k++) {
    clusters_.at(k).addSample(sample, k == optimalIndex, alpha, minimumVariance);
  }
  this->normalizeWeights();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::normalizeWeights() {
  double sum = 0.0;
  for (const Cluster& cluster : clusters_) { sum += cluster.getWeight(); }
  double multiplier = 1.0 / sum;

  for (Cluster& cluster : clusters_) { cluster.getWeight() *= multiplier; }
}

//////////////////////////////////////////////////////////////////////////////////////////////
const std::vector<Cluster>& GMM::getClusters() const { return clusters_; }

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::reset() { clusters_.clear(); }

}  // namespace jpcc::math