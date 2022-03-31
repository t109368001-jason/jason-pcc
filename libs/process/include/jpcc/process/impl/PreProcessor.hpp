#include <jpcc/process/PreProcessor.h>

#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/range/counting_range.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
PreProcessor<PointT>::PreProcessor(PreProcessParameter param) : param_(std::move(param)) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void PreProcessor<PointT>::process(GroupOfFrame&            groupOfFrame,
                                   const GroupOfFrameMapPtr removedMap,
                                   const bool               parallel) {
  for (const std::string& algorithm : param_.order) {
    GroupOfFrame removed;
    if (removedMap) {
      removed.resize(groupOfFrame.size());
      for (auto& frame : removed) { frame.reset(new Frame()); }
    }
    applyAlgorithm(algorithm, groupOfFrame, removed, parallel);
    if (removedMap) { removedMap->insert_or_assign(algorithm, removed); }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
typename PreProcessor<PointT>::FilterPtr PreProcessor<PointT>::buildFilter(const std::string& algorithm) {
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
    BOOST_THROW_EXCEPTION(std::logic_error("not support algorithm: " + algorithm));
  }
  return filter;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void PreProcessor<PointT>::applyAlgorithm(const std::string& algorithm,
                                          GroupOfFrame&      groupOfFrame,
                                          GroupOfFrame&      removed,
                                          const bool         parallel) {
  auto func = [&](const size_t i) {
    this->applyAlgorithm(algorithm, groupOfFrame.at(i), removed.empty() ? nullptr : removed.at(i));
  };
  const auto range = boost::counting_range<size_t>(0, groupOfFrame.size());
  if (parallel) {
    std::for_each(std::execution::par_unseq, range.begin(), range.end(), func);
  } else {
    std::for_each(range.begin(), range.end(), func);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void PreProcessor<PointT>::applyAlgorithm(const std::string& algorithm, const FramePtr frame, const FramePtr removed) {
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