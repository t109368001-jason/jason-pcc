#include <gtest/gtest.h>

#include <jpcc/math/GMM.h>

#define TOLERANCE 1e-5

namespace jpcc::octree {

using namespace std;
using namespace jpcc::math;

TEST(GMMTest, new_by_3_samples) {
  // given
  constexpr int K = 3;
  vector<float> samples;
  for (size_t k = 0; k < K; k++) { samples.push_back((float)k); }

  // when
  GMM gmm;
  gmm.buildN(samples, K + 1, samples.size(), 0, 1000, {});

  // then
  const vector<Cluster>& clusters = gmm.getClusters();
  EXPECT_EQ(clusters.size(), K + 1);
  for (size_t k = 0; k < K; k++) {
    EXPECT_NEAR(clusters.at(k).getWeight(), 1.0 / K, TOLERANCE);
    EXPECT_NEAR(clusters.at(k).getMean(), k, TOLERANCE);
    EXPECT_NEAR(clusters.at(k).getVariance(), 0, TOLERANCE);
  }
  EXPECT_NEAR(clusters.at(K).getWeight(), 0, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getMean(), 1000, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getVariance(), 0, TOLERANCE);
}

TEST(GMMTest, new_by_3_unique_samples) {
  // given
  constexpr int K = 3;
  vector<float> samples;
  vector<int>   counts;
  for (size_t k = 0; k < K; k++) {
    int count = (rand() % 10) + 1;
    counts.push_back(count);
    for (size_t j = 0; j < count; j++) { samples.push_back((float)k); }
  }
  int totalCount = 0;
  for (size_t k = 0; k < K; k++) { totalCount += counts.at(k); }

  // when
  GMM gmm;
  gmm.buildN(samples, K + 1, samples.size(), 0, 1000, {});

  // then
  const vector<Cluster>& clusters = gmm.getClusters();
  EXPECT_EQ(clusters.size(), K + 1);
  for (size_t k = 0; k < K; k++) {
    EXPECT_NEAR(clusters.at(k).getWeight(), (double)counts.at(k) / (double)totalCount, TOLERANCE);
    EXPECT_NEAR(clusters.at(k).getMean(), k, TOLERANCE);
    EXPECT_NEAR(clusters.at(k).getVariance(), 0, TOLERANCE);
  }
  EXPECT_NEAR(clusters.at(K).getWeight(), 0, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getMean(), 1000, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getVariance(), 0, TOLERANCE);
}

TEST(GMMTest, new_3_cluster_3_variance) {
  // given
  constexpr int        K         = 3;
  const vector<double> means     = {1, 50, 500};
  const vector<double> variances = {1, 10, 100};
  vector<float>        samples;
  for (size_t k = 0; k < K; k++) {
    for (size_t i = 0; i < 100; i++) {
      samples.push_back(means.at(k) - sqrt(variances.at(k)));
      samples.push_back(means.at(k) + sqrt(variances.at(k)));
    }
  }

  // when
  GMM gmm;
  gmm.buildN(samples, K + 1, samples.size(), 0, 1000, {});

  // then
  const vector<Cluster>& clusters = gmm.getClusters();
  EXPECT_EQ(clusters.size(), K + 1);
  for (size_t k = 0; k < K; k++) {
    EXPECT_NEAR(clusters.at(k).getWeight(), 1.0 / K, TOLERANCE);
    EXPECT_NEAR(clusters.at(k).getMean(), means.at(k), TOLERANCE);
    EXPECT_NEAR(clusters.at(k).getVariance(), variances.at(k), TOLERANCE);
  }
  EXPECT_NEAR(clusters.at(K).getWeight(), 0, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getMean(), 1000, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getVariance(), 0, TOLERANCE);
}

TEST(GMMTest, getOptimalModelIndex) {
  // given
  constexpr int        K         = 3;
  const vector<double> means     = {1, 50, 500};
  const vector<double> variances = {1, 10, 100};
  vector<float>        samples;
  for (size_t k = 0; k < K; k++) {
    for (size_t i = 0; i < 100; i++) {
      samples.push_back(means.at(k) - variances.at(k));
      samples.push_back(means.at(k) + variances.at(k));
    }
  }
  GMM gmm;
  gmm.buildN(samples, K + 1, samples.size(), 0, 1000, {});

  // then
  for (size_t k = 0; k < K; k++) {
    const int& index = gmm.getOptimalModelIndex(means.at(k));
    EXPECT_EQ(index, k);
  }
}

TEST(GMMTest, updateModel) {
  // given
  constexpr int        K            = 3;
  const vector<double> means        = {1, 50, 500};
  const vector<double> variances    = {1, 10, 100};
  constexpr double     newVariance0 = 5;
  vector<float>        samples;
  for (size_t k = 0; k < K; k++) {
    for (size_t i = 0; i < 100; i++) {
      samples.push_back(means.at(k) - sqrt(variances.at(k)));
      samples.push_back(means.at(k) + sqrt(variances.at(k)));
    }
  }
  GMM gmm;
  gmm.buildN(samples, K + 1, samples.size(), 0, 1000, {});

  // when
  vector<float> newSamples;
  for (size_t i = 0; i < 10; i++) {
    newSamples.push_back(means.at(0) - sqrt(newVariance0));
    newSamples.push_back(means.at(0) + sqrt(newVariance0));
  }
  for (const auto& sample : newSamples) { gmm.updateModel(sample, 0.05, 0); }

  // then
  const vector<Cluster>& clusters = gmm.getClusters();
  EXPECT_EQ(clusters.size(), K + 1);
  EXPECT_NEAR(clusters.at(0).getWeight(), 0.7610093851, TOLERANCE);
  EXPECT_NEAR(clusters.at(0).getMean(), 1.000249155528542, TOLERANCE);
  EXPECT_NEAR(clusters.at(0).getVariance(), 1.1445250926215722, TOLERANCE);
  EXPECT_NEAR(clusters.at(1).getWeight(), 0.1194953075, TOLERANCE);
  EXPECT_NEAR(clusters.at(1).getMean(), 50, TOLERANCE);
  EXPECT_NEAR(clusters.at(1).getVariance(), 9.9999972267542034, TOLERANCE);
  EXPECT_NEAR(clusters.at(2).getWeight(), 0.1194953075, TOLERANCE);
  EXPECT_NEAR(clusters.at(2).getMean(), 499.9999921, TOLERANCE);
  EXPECT_NEAR(clusters.at(2).getVariance(), 100, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getWeight(), 0, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getMean(), 1000, TOLERANCE);
  EXPECT_NEAR(clusters.at(K).getVariance(), 0, TOLERANCE);
}

}  // namespace jpcc::octree
