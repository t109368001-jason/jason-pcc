#include <jpcc/process/PreProcessor.h>

#include <execution>
#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/range/counting_range.hpp>

#include <dependencies/nanoflann/KDTreeVectorOfVectorsAdaptor.h>

#include "jpcc/process/Process.h"

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
void PreProcessor::applyAlgorithm(const std::string& algorithm,
                                  GroupOfFrame&      groupOfFrame,
                                  GroupOfFrame&      removed,
                                  const bool         parallel) const {
  if (!parallel) {
    for (size_t i = 0; i < groupOfFrame.size(); i++) {
      this->applyAlgorithm(algorithm, groupOfFrame[i], removed.empty() ? nullptr : removed[i]);
    }
  } else {
    auto func = [&](const size_t i) {
      this->applyAlgorithm(algorithm, groupOfFrame[i], removed.empty() ? nullptr : removed[i]);
    };
    const auto range = boost::counting_range<size_t>(0, groupOfFrame.size());
    std::for_each(std::execution::par_unseq, range.begin(), range.end(), func);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PreProcessor::applyAlgorithm(const std::string& algorithm, const FramePtr& frame, const FramePtr& removed) const {
  Indices indices;
  Indices removedIndices;
  indices.reserve(frame->getPointCount());
  removedIndices.reserve(frame->getPointCount());
  if (algorithm == RADIUS_OUTLIER_REMOVAL_OPT_PREFIX) {
    std::vector<std::pair<Index, double> >    indicesDists;
    nanoflann::RadiusResultSet<double, Index> resultSet(param_.radiusOutlierRemoval.radius, indicesDists);

    KDTreeVectorOfVectorsAdaptor<Frame, double> kdtree(3, *frame, 10);

    for (Index i = 0; i < frame->getPointCount(); i++) {
      auto&             point       = (*frame)[i];
      pcc::Vec3<double> pointDouble = point;
      resultSet.init();
      bool ret = kdtree.index->radiusSearchCustomCallback(&pointDouble[0], resultSet, nanoflann::SearchParams(10));
      if (indicesDists.size() >= param_.radiusOutlierRemoval.minNeighborsInRadius) {
        indices.push_back(i);
      } else {
        removedIndices.push_back(i);
      }
    }
  } else if (algorithm == JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX) {
    for (Index i = 0; i < frame->getPointCount(); i++) {
      bool predict = param_.jpccConditionalRemovalParameter.condition.predict((*frame)[i]);
      if (predict) {
        indices.push_back(i);
      } else {
        removedIndices.push_back(i);
      }
    }
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("not support algorithm: " + algorithm));
  }
  auto _frame = jpcc::make_shared<Frame>();
  frame->subset(*_frame, indices);
  if (removed) { frame->subset(*removed, removedIndices); }
}

}  // namespace jpcc::process