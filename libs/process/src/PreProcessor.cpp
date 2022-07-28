#include <jpcc/process/PreProcessor.h>

#include <execution>
#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/range/counting_range.hpp>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <jpcc/process/JPCCConditionalRemoval.h>
#include <jpcc/process/Process.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////

PreProcessor::PreProcessor(PreProcessParameter param) : param_(std::move(param)) {}

//////////////////////////////////////////////////////////////////////////////////////////////

void PreProcessor::process(GroupOfFrame&             groupOfFrame,
                           const GroupOfFrameMapPtr& removedMap,
                           const bool                parallel) const {
  for (const std::string& algorithm : param_.order) {
    GroupOfFrame removed;
    if (removedMap) {
      removed.resize(groupOfFrame.size());
      for (auto& frame : removed) { frame = jpcc::make_shared<Frame>(); }
    }
    applyAlgorithm(algorithm, groupOfFrame, removed, parallel);
    if (removedMap) { removedMap->insert_or_assign(algorithm, removed); }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////

typename PreProcessor::FilterPtr PreProcessor::buildFilter(const std::string& algorithm) const {
  FilterPtr filter;
  if (algorithm == RADIUS_OUTLIER_REMOVAL_OPT_PREFIX) {
    auto radiusOutlierRemoval = jpcc::make_shared<pcl::RadiusOutlierRemoval<PointXYZINormal>>();
    radiusOutlierRemoval->setRadiusSearch(param_.radiusOutlierRemoval.radius);
    radiusOutlierRemoval->setMinNeighborsInRadius(param_.radiusOutlierRemoval.minNeighborsInRadius);
    filter = radiusOutlierRemoval;
  } else if (algorithm == STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX) {
    auto statisticalOutlierRemoval = jpcc::make_shared<pcl::StatisticalOutlierRemoval<PointXYZINormal>>();
    statisticalOutlierRemoval->setMeanK(param_.statisticalOutlierRemoval.meanK);
    statisticalOutlierRemoval->setStddevMulThresh(param_.statisticalOutlierRemoval.stddevMulThresh);
    filter = statisticalOutlierRemoval;
  } else if (algorithm == JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX) {
    filter = jpcc::make_shared<JPCCConditionalRemoval>(param_.jpccConditionalRemovalParameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("not support algorithm: " + algorithm));
  }
  return filter;
}

//////////////////////////////////////////////////////////////////////////////////////////////

void PreProcessor::applyAlgorithm(const std::string& algorithm,
                                  GroupOfFrame&      groupOfFrame,
                                  GroupOfFrame&      removed,
                                  const bool         parallel) const {
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

void PreProcessor::applyAlgorithm(const std::string& algorithm, const FramePtr& frame, const FramePtr& removed) const {
  FilterPtr filter = buildFilter(algorithm);
  filter->setInputCloud(frame);
  if (!removed) {
    filter->filter(*frame);
  } else {
    removed->clear();
    auto indices = jpcc::make_shared<Indices>();
    filter->filter(*indices);
    split(frame, indices, frame, removed);
  }
}

}  // namespace jpcc::process