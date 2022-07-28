#include <jpcc/math/GMM.h>

#include <numeric>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
GMM::GMM(vector<float>&            samples,
         const int                 K,
         const double              minimumVariance,
         const std::vector<float>& seedCentroids,
         const std::vector<float>& alternateCentroids) {
  build(samples, K, minimumVariance, seedCentroids, alternateCentroids);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void GMM::build(vector<float>&            samples,
                const int                 K,
                const double              minimumVariance,
                const std::vector<float>& seedCentroids,
                const std::vector<float>& alternateCentroids) {
  size_t k;

  vector<float> centroids;
  for (const auto& seedCentroid : seedCentroids) {
    centroids.push_back(seedCentroid);
    samples.push_back(seedCentroid);
  }
  vector<vector<float>> clusters(K);
  // K-Means
  // get centroids
  std::vector<float> uniqueSamples;
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
      float sample = samples.at(rand() % samples.size());
      bool  exists = false;
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
      float sample = uniqueSamples.at(i);
      bool  exists = false;
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

  std::vector<float> previousCentroids;
  for (const auto& centroid : centroids) {
    assert(!isnan(centroid));
    previousCentroids.push_back(centroid);
  }

  bool isConverged = false;
  while (!isConverged) {
    for (k = 0; k < K; k++) { clusters.at(k).clear(); }

    for (const auto& sample : samples) {
      size_t minIndex    = 0;
      float  minDistance = abs(sample - centroids.at(0));
      for (k = 1; k < K; k++) {
        float distance = abs(sample - centroids.at(k));
        if (distance < minDistance) {
          minIndex    = k;
          minDistance = distance;
        }
      }
      clusters.at(minIndex).push_back(sample);
    }

    for (k = 0; k < K; k++) {
      if (clusters.at(k).empty()) {
        for (int k2 = 0; k2 < K; k2++) {
          if (clusters.at(k2).size() < 2) { continue; }
          std::vector<float> uniques;
          for (const auto& sample : clusters.at(k2)) {
            bool exists = false;
            for (const auto& unique : uniques) {
              if (sample == unique) {
                exists = true;
                break;
              }
            }
            if (!exists) {
              uniques.push_back(sample);
              if (uniques.size() > 1) { break; }
            }
          }
          if (uniques.size() > 1) {
            centroids.at(k) = uniques.at(0);
            break;
          }
        }
        continue;
      }
      const float sum = accumulate(clusters.at(k).begin(), clusters.at(k).end(), float{0});
      centroids.at(k) = static_cast<float>(sum / static_cast<double>(clusters.at(k).size()));
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
[[nodiscard]] double GMM::getProbability(float sample) {
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
    double probability = cluster.getWeight() * cluster.getProbability(static_cast<float>(cluster.getMean()));
    if (cluster.getMean() < 0) {
      totalProbability -= probability;
    } else {
      totalProbability += probability;
    }
  }
  return totalProbability;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pair<size_t, double> GMM::getOptimalModelIndex(const float sample) const {
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
void GMM::updateModel(float sample, const double alpha, const double minimumVariance) {
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