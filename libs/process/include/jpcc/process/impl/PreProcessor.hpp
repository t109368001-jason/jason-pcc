#include <jpcc/process/PreProcessor.h>

#include <boost/algorithm/string.hpp>
#include <boost/range/counting_range.hpp>
#include <utility>

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
    if (boost::iequals(algorithm, RADIUS_OUTLIER_REMOVAL_OPT_PREFIX)) {
      GroupOfFramePtr groupOfFramePtr;
      if (removed) {
        groupOfFramePtr.reset(new GroupOfFrame());
        groupOfFramePtr->resize(groupOfFrame.size());
        for (auto& frame : *groupOfFramePtr) { frame.reset(new Frame()); }
      }
      radiusOutlierRemoval(groupOfFrame, groupOfFramePtr, parallel);
      if (removed) { removed->insert_or_assign(RADIUS_OUTLIER_REMOVAL_OPT_PREFIX, groupOfFramePtr); }
    }
    if (boost::iequals(algorithm, STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX)) {
      GroupOfFramePtr groupOfFramePtr;
      if (removed) {
        groupOfFramePtr.reset(new GroupOfFrame());
        groupOfFramePtr->resize(groupOfFrame.size());
        for (auto& frame : *groupOfFramePtr) { frame.reset(new Frame()); }
      }
      statisticalOutlierRemoval(groupOfFrame, groupOfFramePtr, parallel);
      if (removed) { removed->insert_or_assign(STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX, groupOfFramePtr); }
    }
  }
}

template <class PointT>
void PreProcessor<PointT>::radiusOutlierRemoval(FramePtr& frame, FramePtr removed) {
  pcl::RadiusOutlierRemoval<PointT> radiusOutlierRemoval;
  radiusOutlierRemoval.setRadiusSearch(param_.radiusOutlierRemoval.radius);
  radiusOutlierRemoval.setMinNeighborsInRadius(param_.radiusOutlierRemoval.minNeighborsInRadius);
  //  radiusOutlierRemoval.setKeepOrganized(true);
  radiusOutlierRemoval.setInputCloud(frame);
  if (!removed) {
    radiusOutlierRemoval.filter(*frame);
  } else {
    removed->clear();
    shared_ptr<pcl::Indices> indices(new pcl::Indices());
    radiusOutlierRemoval.filter(*indices);
    pcl::ExtractIndices<PointT> extractIndices;
    extractIndices.setInputCloud(frame);
    extractIndices.setIndices(indices);
    extractIndices.setNegative(true);
    extractIndices.filter(*removed);
    extractIndices.setNegative(false);
    extractIndices.filter(*frame);
  }
}

template <class PointT>
void PreProcessor<PointT>::radiusOutlierRemoval(GroupOfFrame& groupOfFrame, GroupOfFramePtr removed, bool parallel) {
  auto range = boost::counting_range<size_t>(0, groupOfFrame.size());
  if (parallel) {
    std::for_each(std::execution::par_unseq, range.begin(), range.end(), [&](size_t i) {
      radiusOutlierRemoval(groupOfFrame.at(i), removed != nullptr ? removed->at(i) : nullptr);
    });
  } else {
    std::for_each(range.begin(), range.end(), [&](size_t i) {
      radiusOutlierRemoval(groupOfFrame.at(i), removed != nullptr ? removed->at(i) : nullptr);
    });
  }
}

template <class PointT>
void PreProcessor<PointT>::statisticalOutlierRemoval(FramePtr& frame, FramePtr removed) {
  pcl::StatisticalOutlierRemoval<PointT> statisticalOutlierRemoval;
  statisticalOutlierRemoval.setMeanK(param_.statisticalOutlierRemoval.meanK);
  statisticalOutlierRemoval.setStddevMulThresh(param_.statisticalOutlierRemoval.stddevMulThresh);
  //  radiusOutlierRemoval.setKeepOrganized(true);
  statisticalOutlierRemoval.setInputCloud(frame);
  if (!removed) {
    statisticalOutlierRemoval.filter(*frame);
  } else {
    removed->clear();
    shared_ptr<pcl::Indices> indices(new pcl::Indices());
    statisticalOutlierRemoval.filter(*indices);
    pcl::ExtractIndices<PointT> extractIndices;
    extractIndices.setInputCloud(frame);
    extractIndices.setIndices(indices);
    extractIndices.setNegative(true);
    extractIndices.filter(*removed);
    extractIndices.setNegative(false);
    extractIndices.filter(*frame);
  }
}

template <class PointT>
void PreProcessor<PointT>::statisticalOutlierRemoval(GroupOfFrame&                 groupOfFrame,
                                                     PreProcessor::GroupOfFramePtr removed,
                                                     bool                          parallel) {
  auto range = boost::counting_range<size_t>(0, groupOfFrame.size());
  if (parallel) {
    std::for_each(std::execution::par_unseq, range.begin(), range.end(), [&](size_t i) {
      statisticalOutlierRemoval(groupOfFrame.at(i), removed != nullptr ? removed->at(i) : nullptr);
    });
  } else {
    std::for_each(range.begin(), range.end(), [&](size_t i) {
      statisticalOutlierRemoval(groupOfFrame.at(i), removed != nullptr ? removed->at(i) : nullptr);
    });
  }
}

}  // namespace jpcc::process