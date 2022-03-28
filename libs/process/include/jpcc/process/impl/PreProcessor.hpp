#include <jpcc/process/PreProcessor.h>

#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/range/counting_range.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace jpcc::process {

using namespace std;
using namespace jpcc;

template <class PointT>
PreProcessor<PointT>::PreProcessor(PreProcessParameter param) : param_(std::move(param)) {}

template <class PointT>
void PreProcessor<PointT>::process(GroupOfFrame& groupOfFrame, GroupOfFrameMapPtr removed, bool parallel) {
  for (const string& algorithm : param_.order) {
    GroupOfFramePtr groupOfFramePtr;
    if (removed) {
      groupOfFramePtr.reset(new GroupOfFrame());
      groupOfFramePtr->resize(groupOfFrame.size());
      for (auto& frame : *groupOfFramePtr) { frame.reset(new Frame()); }
    }
    applyAlgorithm(algorithm, groupOfFrame, groupOfFramePtr, parallel);
    if (removed) { removed->insert_or_assign(algorithm, groupOfFramePtr); }
  }
}

template <class PointT>
typename PreProcessor<PointT>::FilterPtr PreProcessor<PointT>::buildFilter(const string& algorithm) {
  FilterPtr filter;
  if (algorithm == RADIUS_OUTLIER_REMOVAL_OPT_PREFIX) {
    typename pcl::RadiusOutlierRemoval<PointT>::Ptr radiusOutlierRemoval(new pcl::RadiusOutlierRemoval<PointT>());
    radiusOutlierRemoval->setRadiusSearch(param_.radiusOutlierRemoval.radius);
    radiusOutlierRemoval->setMinNeighborsInRadius(param_.radiusOutlierRemoval.minNeighborsInRadius);
    filter = radiusOutlierRemoval;
  } else if (algorithm == STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX) {
    typename pcl::StatisticalOutlierRemoval<PointT>::Ptr statisticalOutlierRemoval(
        new pcl::StatisticalOutlierRemoval<PointT>());
    statisticalOutlierRemoval->setMeanK(param_.statisticalOutlierRemoval.meanK);
    statisticalOutlierRemoval->setStddevMulThresh(param_.statisticalOutlierRemoval.stddevMulThresh);
    filter = statisticalOutlierRemoval;
  } else {
    BOOST_THROW_EXCEPTION(logic_error("not support algorithm: " + algorithm));
  }
  return filter;
}

template <class PointT>
void PreProcessor<PointT>::applyAlgorithm(const string&                 algorithm,
                                          GroupOfFrame&                 groupOfFrame,
                                          PreProcessor::GroupOfFramePtr removed,
                                          bool                          parallel) {
  auto func = [&](size_t i) {
    this->applyAlgorithm(algorithm, groupOfFrame.at(i), removed ? removed->at(i) : nullptr);
  };
  auto range = boost::counting_range<size_t>(0, groupOfFrame.size());
  if (parallel) {
    std::for_each(std::execution::par_unseq, range.begin(), range.end(), func);
  } else {
    std::for_each(range.begin(), range.end(), func);
  }
}

template <class PointT>
void PreProcessor<PointT>::applyAlgorithm(const string& algorithm, FramePtr frame, FramePtr removed) {
  FilterPtr filter = buildFilter(algorithm);
  filter->setInputCloud(frame);
  if (!removed) {
    filter->filter(*frame);
  } else {
    removed->clear();
    shared_ptr<pcl::Indices> indices(new pcl::Indices());
    filter->filter(*indices);
    pcl::ExtractIndices<PointT> extractIndices;
    extractIndices.setInputCloud(frame);
    extractIndices.setIndices(indices);
    extractIndices.setNegative(true);
    extractIndices.filter(*removed);
    extractIndices.setNegative(false);
    extractIndices.filter(*frame);
  }
}

}  // namespace jpcc::process