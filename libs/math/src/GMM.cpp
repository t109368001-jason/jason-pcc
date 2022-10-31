#include <jpcc/math/GMM.h>

#include <random>
#include <iterator>

#include <jpcc/math/KMeans.h>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::buildN(vector<int32_t>&     samples,
                 const int            K,
                 const int            N,
                 const double         minimumVariance,
                 const int32_t        nullSample,
                 const vector<float>& alternateCentroids) {
  int    K_ = K - 1;
  size_t k;

  vector<float> centroids;

  // get centroids
  vector<int32_t> uniqueSamples;
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
      std::random_device              rd;
      std::mt19937                    gen(rd());
      std::uniform_int_distribution<> dis(0, std::distance(samples.begin(), samples.end()) - 1);
      auto                            sampleIter = samples.begin();
      std::advance(sampleIter, dis(gen));

      bool exists = false;
      for (const auto& centroid : centroids) {
        if (std::abs(float(*sampleIter) - centroid) < 0.00001) {
          exists = true;
          break;
        }
      }
      if (!exists) { centroids.push_back(float(*sampleIter)); }
    }
  } else {
    for (size_t i = 0; i < uniqueSamples.size() && centroids.size() < K_; i++) {
      const int32_t sample = uniqueSamples[i];
      bool          exists = false;
      for (float centroid : centroids) {
        if (std::abs(float(sample) - centroid) < 0.00001) {
          exists = true;
          break;
        }
      }
      if (!exists) { centroids.push_back(float(sample)); }
    }
    for (size_t i = 0; i < alternateCentroids.size() && centroids.size() < K_; i++) {
      centroids.push_back(alternateCentroids[i]);
      samples.push_back(int32_t(alternateCentroids[i]));
    }
  }

  vector<vector<int32_t>> clusters(K_);
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
[[nodiscard]] double GMM::getProbability(int32_t sample) {
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
size_t GMM::getOptimalModelIndex(const int32_t sample) const {
  assert(!isnan(sample));
  size_t optimalIndex       = 0;
  double optimalProbability = clusters_.front().getWeight() * clusters_.front().getProbability(sample);
  for (size_t k = 1; k < clusters_.size(); k++) {
    double probability = clusters_[k].getWeight() * clusters_[k].getProbability(sample);
    if (probability >= optimalProbability) {
      optimalIndex       = k;
      optimalProbability = probability;
    }
  }
  return optimalIndex;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::updateModel(int32_t sample, const double alpha, const double minimumVariance) {
  assert(!isnan(sample));
  const size_t optimalIndex = getOptimalModelIndex(sample);
  for (size_t k = 0; k < clusters_.size(); k++) {
    clusters_[k].addSample(sample, k == optimalIndex, alpha, minimumVariance);
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