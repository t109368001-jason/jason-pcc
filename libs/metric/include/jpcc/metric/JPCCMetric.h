#pragma once

#include <map>
#include <set>
#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/common/ScopeStopwatch.h>
#include <jpcc/metric/JPCCMetricParameter.h>

namespace jpcc::metric {

class JPCCMetric {
 public:
  using FrameNumber = decltype(pcl::PCLHeader::seq);

 protected:
  const JPCCMetricParameter&                              parameter_;
  std::set<FrameNumber>                                   frameNumberSet_;
  std::set<std::string>                                   pointsNameSet_;
  std::set<std::string>                                   bytesNameSet_;
  std::set<std::string>                                   clockNameSet_;
  std::map<FrameNumber, std::map<std::string, uint64_t>>  pointsMapMap_;
  std::map<FrameNumber, std::map<std::string, uint64_t>>  bytesMapMap_;
  std::map<FrameNumber, std::map<std::string, Stopwatch>> clockMapMap_;

 public:
  JPCCMetric(const JPCCMetricParameter& parameter);

  template <typename PointT>
  void addPoints(const std::string& name, const FramePtr<PointT>& frame);

  template <typename PointT>
  void addPoints(const std::string& name, const GroupOfFrame<PointT>& frames);

  void addBytes(const std::string& name, FrameNumber frameNumber, uint64_t bytes);

  ScopeStopwatch start(const std::string& name, FrameNumber frameNumber);

  void writeAndShow();
};

}  // namespace jpcc::metric

#include <jpcc/metric/impl/JPCCMetric.hpp>
