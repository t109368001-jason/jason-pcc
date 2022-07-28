#include <jpcc/math/GMM.h>

#include <numeric>

using namespace std;

namespace jpcc::math {

//////////////////////////////////////////////////////////////////////////////////////////////
void kmeans(const std::vector<float>&        samples,
            int                              K,
            std::vector<float>&              centroids,
            std::vector<std::vector<float>>& clusters) {
  clusters.resize(K);

  std::vector<float> previousCentroids;
  for (const auto& centroid : centroids) {
    assert(!isnan(centroid));
    previousCentroids.push_back(centroid);
  }

  int  k;
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
}

}  // namespace jpcc::math