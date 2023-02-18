#include <jpcc/math/KMeans.h>

#include <numeric>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
void kmeans(const std::vector<Intensity>&        samples,
            int                                  K,
            std::vector<float>&                  centroids,
            std::vector<std::vector<Intensity>>& clusters) {
  clusters.resize(K);

  std::vector<float> previousCentroids;
  for (const auto& centroid : centroids) {
    assert(!isnan(centroid));
    previousCentroids.push_back(centroid);
  }

  int  k;
  bool isConverged = false;
  while (!isConverged) {
    for (k = 0; k < K; k++) { clusters[k].clear(); }

    for (const auto& sample : samples) {
      size_t minIndex    = 0;
      float  minDistance = abs(static_cast<float>(sample) - centroids.front());
      for (k = 1; k < K; k++) {
        const float distance = abs(static_cast<float>(sample) - centroids[k]);
        if (distance < minDistance) {
          minIndex    = k;
          minDistance = distance;
        }
      }
      clusters[minIndex].push_back(sample);
    }

    for (k = 0; k < K; k++) {
      if (clusters[k].empty()) {
        for (int k2 = 0; k2 < K; k2++) {
          if (clusters[k2].size() < 2) { continue; }
          std::vector<Intensity> uniques;
          for (const auto& sample : clusters[k2]) {
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
            centroids[k] = static_cast<float>(uniques.front());
            break;
          }
        }
        continue;
      }
      const double sum = accumulate(clusters[k].begin(), clusters[k].end(), double{0.0});
      centroids[k]     = static_cast<float>(sum / static_cast<double>(clusters[k].size()));
    }

    isConverged = true;
    for (k = 0; k < K; k++) {
      if (centroids[k] != previousCentroids[k]) { isConverged = false; }
    }
    copy(centroids.begin(), centroids.end(), previousCentroids.begin());
  }
}

}  // namespace jpcc::math