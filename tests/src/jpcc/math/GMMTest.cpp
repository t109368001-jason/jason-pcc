#include <gtest/gtest.h>

#include <jpcc/math/GMM.h>

#define TOLERANCE 1e-5

namespace jpcc::octree {

using namespace std;
using namespace jpcc::math;

TEST(GMMTest, new_by_3_samples) {
  // given
  constexpr int        K = 3;
  vector<GMM::SampleT> samples;
  for (size_t k = 0; k < K; k++) { samples.push_back((GMM::SampleT)k); }
  vector<GMM::SampleT> alternateCentroids;

  // when
  const GMM gmm(samples, K, 0.05, 0, alternateCentroids);

  // then
  const vector<Cluster::Ptr>& clusters = gmm.getClusters();
  EXPECT_EQ(clusters.size(), K);
  for (size_t k = 0; k < K; k++) {
    EXPECT_NEAR(clusters.at(k)->getWeight(), 1.0 / K, TOLERANCE);
    EXPECT_NEAR(clusters.at(k)->getMean(), k, TOLERANCE);
    EXPECT_NEAR(clusters.at(k)->getVariance(), 0, TOLERANCE);
  }
}

TEST(GMMTest, new_by_3_unique_samples) {
  // given
  constexpr int        K = 3;
  vector<GMM::SampleT> samples;
  vector<int>          counts;
  for (size_t k = 0; k < K; k++) {
    int count = (rand() % 10) + 1;
    counts.push_back(count);
    for (size_t j = 0; j < count; j++) { samples.push_back((GMM::SampleT)k); }
  }
  vector<GMM::SampleT> alternateCentroids;
  int                  totalCount = 0;
  for (size_t k = 0; k < K; k++) { totalCount += counts.at(k); }

  // when
  const GMM gmm(samples, K, 0.05, 0, alternateCentroids);

  // then
  const vector<Cluster::Ptr>& clusters = gmm.getClusters();
  EXPECT_EQ(clusters.size(), K);
  for (size_t k = 0; k < K; k++) {
    EXPECT_NEAR(clusters.at(k)->getWeight(), (double)counts.at(k) / (double)totalCount, TOLERANCE);
    EXPECT_NEAR(clusters.at(k)->getMean(), k, TOLERANCE);
    EXPECT_NEAR(clusters.at(k)->getVariance(), 0, TOLERANCE);
  }
}

TEST(GMMTest, new_3_cluster_3_variance) {
  // given
  constexpr int        K         = 3;
  const vector<double> means     = {1, 50, 500};
  const vector<double> variances = {1, 10, 100};
  vector<GMM::SampleT> samples;
  for (size_t k = 0; k < K; k++) {
    for (size_t i = 0; i < 100; i++) {
      samples.push_back(means.at(k) - sqrt(variances.at(k)));
      samples.push_back(means.at(k) + sqrt(variances.at(k)));
    }
  }
  vector<GMM::SampleT> alternateCentroids;

  // when
  const GMM gmm(samples, K, 0.05, 0, alternateCentroids);

  // then
  const vector<Cluster::Ptr>& clusters = gmm.getClusters();
  EXPECT_EQ(clusters.size(), K);
  for (size_t k = 0; k < K; k++) {
    EXPECT_NEAR(clusters.at(k)->getWeight(), 1.0 / K, TOLERANCE);
    EXPECT_NEAR(clusters.at(k)->getMean(), means.at(k), TOLERANCE);
    EXPECT_NEAR(clusters.at(k)->getVariance(), variances.at(k), TOLERANCE);
  }
}

TEST(GMMTest, getOptimalModelIndex) {
  // given
  constexpr int        K         = 3;
  const vector<double> means     = {1, 50, 500};
  const vector<double> variances = {1, 10, 100};
  vector<GMM::SampleT> samples;
  for (size_t k = 0; k < K; k++) {
    for (size_t i = 0; i < 100; i++) {
      samples.push_back(means.at(k) - variances.at(k));
      samples.push_back(means.at(k) + variances.at(k));
    }
  }
  vector<GMM::SampleT> alternateCentroids;
  const GMM            gmm(samples, K, 0.05, 0, alternateCentroids);

  // then
  for (size_t k = 0; k < K; k++) {
    const pair<size_t, double>& result = gmm.getOptimalModelIndex(means.at(k));
    EXPECT_NEAR(result.first, k, TOLERANCE);
    EXPECT_NEAR(result.second, 1 / sqrt(M_PI * 2) / variances.at(k), TOLERANCE);
  }
}

TEST(GMMTest, updateModel) {
  // given
  constexpr int        K            = 3;
  const vector<double> means        = {1, 50, 500};
  const vector<double> variances    = {1, 10, 100};
  constexpr double     newVariance0 = 5;
  vector<GMM::SampleT> samples;
  for (size_t k = 0; k < K; k++) {
    for (size_t i = 0; i < 100; i++) {
      samples.push_back(means.at(k) - sqrt(variances.at(k)));
      samples.push_back(means.at(k) + sqrt(variances.at(k)));
    }
  }
  vector<GMM::SampleT> alternateCentroids;
  GMM                  gmm(samples, K, 0.05, 0, alternateCentroids);

  // when
  vector<GMM::SampleT> newSamples;
  for (size_t i = 0; i < 10; i++) {
    newSamples.push_back(means.at(0) - sqrt(newVariance0));
    newSamples.push_back(means.at(0) + sqrt(newVariance0));
  }
  gmm.updateModel(newSamples);

  // then
  const vector<Cluster::Ptr>& clusters = gmm.getClusters();
  EXPECT_EQ(clusters.size(), K);
  EXPECT_NEAR(clusters.at(0)->getWeight(), 0.7610093851, TOLERANCE);
  EXPECT_NEAR(clusters.at(0)->getMean(), 1.000249155528542, TOLERANCE);
  EXPECT_NEAR(clusters.at(0)->getVariance(), 1.1445250926215722, TOLERANCE);
  EXPECT_NEAR(clusters.at(1)->getWeight(), 0.1194953075, TOLERANCE);
  EXPECT_NEAR(clusters.at(1)->getMean(), 50, TOLERANCE);
  EXPECT_NEAR(clusters.at(1)->getVariance(), 9.9999972267542034, TOLERANCE);
  EXPECT_NEAR(clusters.at(2)->getWeight(), 0.1194953075, TOLERANCE);
  EXPECT_NEAR(clusters.at(2)->getMean(), 499.9999921, TOLERANCE);
  EXPECT_NEAR(clusters.at(2)->getVariance(), 100, TOLERANCE);
}

}  // namespace jpcc::octree
