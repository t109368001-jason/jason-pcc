#include <jpcc/math/GMM.h>

#include <numeric>

#include <jpcc/math/KMeans.h>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::buildN(vector<float>&       samples,
                 const int            K,
                 const int            N,
                 const double         minimumVariance,
                 const float          nullSample,
                 const vector<float>& alternateCentroids) {
  int    K_ = K - 1;
  size_t k;

  vector<float> centroids;

  // get centroids
  vector<float> uniqueSamples;
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
  if (uniqueSamples.size() >= K_) {
    while (centroids.size() < K_) {
      const float sample = samples.at(rand() % samples.size());
      bool        exists = false;
      for (const auto& centroid : centroids) {
        if (sample == centroid) {
          exists = true;
          break;
        }
      }
      if (!exists) { centroids.push_back(sample); }
    }
  } else {
    for (size_t i = 0; i < uniqueSamples.size() && centroids.size() < K_; i++) {
      const float sample = uniqueSamples.at(i);
      bool        exists = false;
      for (float centroid : centroids) {
        if (sample == centroid) {
          exists = true;
          break;
        }
      }
      if (!exists) { centroids.push_back(sample); }
    }
    for (size_t i = 0; i < alternateCentroids.size() && centroids.size() < K_; i++) {
      centroids.push_back(alternateCentroids.at(i));
      samples.push_back(alternateCentroids.at(i));
    }
  }

  vector<vector<float>> clusters(K_);
  kmeans(samples, K_, centroids, clusters);

  clusters_.emplace_back(nullSample, (1 - samples.size() / N), minimumVariance);
  for (const auto& cluster : clusters) {
    const double weight = static_cast<double>(cluster.size()) / static_cast<double>(N);
    clusters_.emplace_back(cluster, weight, minimumVariance);
  }
  sort(clusters_.begin(), clusters_.end());
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool GMM::isBuilt() const { return !clusters_.empty(); }

//////////////////////////////////////////////////////////////////////////////////////////////
[[nodiscard]] double GMM::getProbability(float sample) {
  assert(!isnan(sample));
  double totalProbability = 0.0;

  for (const auto& cluster : clusters_) {
    const double probability = cluster.getWeight() * cluster.getProbability(sample);
    if (cluster.getMean() < 0) {
      totalProbability -= probability;
    } else {
      totalProbability += probability;
    }
  }
  assert(!isnan(totalProbability));
  return totalProbability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
[[nodiscard]] double GMM::getStaticProbability() {
  double totalProbability = 0.0;

  for (const auto& cluster : clusters_) {
    const double probability = cluster.getWeight() * cluster.getStaticProbability();
    if (cluster.getMean() < 0) {
      totalProbability -= probability;
    } else {
      totalProbability += probability;
    }
  }
  assert(!isnan(totalProbability));
  return totalProbability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
size_t GMM::getOptimalModelIndex(const float sample) const {
  assert(!isnan(sample));
  size_t optimalIndex       = 0;
  double optimalProbability = clusters_.at(0).getWeight() * clusters_.at(0).getProbability(sample);
  for (size_t k = 1; k < clusters_.size(); k++) {
    double probability = clusters_.at(k).getWeight() * clusters_.at(k).getProbability(sample);
    if (probability >= optimalProbability) {
      optimalIndex       = k;
      optimalProbability = probability;
    }
  }
  return optimalIndex;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::updateModel(float sample, const double alpha, const double minimumVariance) {
  assert(!isnan(sample));
  const size_t optimalIndex = getOptimalModelIndex(sample);
  for (size_t k = 0; k < clusters_.size(); k++) {
    clusters_.at(k).addSample(sample, k == optimalIndex, alpha, minimumVariance);
  }
  this->normalizeWeights();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::normalizeWeights() {
  double sum = 0.0;
  for (const Cluster& cluster : clusters_) { sum += cluster.getWeight(); }

  for (Cluster& cluster : clusters_) { cluster.getWeight() /= sum; }
}

//////////////////////////////////////////////////////////////////////////////////////////////
const vector<Cluster>& GMM::getClusters() const { return clusters_; }

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::reset() { clusters_.clear(); }

}  // namespace jpcc::math