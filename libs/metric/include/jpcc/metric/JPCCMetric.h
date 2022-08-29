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
  std::set<std::string>                                   c2cMSENameSet_;
  std::set<std::string>                                   c2cPSNRNameSet_;
  std::set<std::string>                                   c2pMSENameSet_;
  std::set<std::string>                                   c2pPSNRNameSet_;
  std::set<std::string>                                   clockNameSet_;
  std::map<FrameNumber, std::map<std::string, uint64_t>>  pointsMapMap_;
  std::map<FrameNumber, std::map<std::string, uint64_t>>  bytesMapMap_;
  std::map<FrameNumber, std::map<std::string, double>>    c2cMSEMapMap_;
  std::map<FrameNumber, std::map<std::string, double>>    c2cPSNRMapMap_;
  std::map<FrameNumber, std::map<std::string, double>>    c2pMSEMapMap_;
  std::map<FrameNumber, std::map<std::string, double>>    c2pPSNRMapMap_;
  std::map<FrameNumber, std::map<std::string, Stopwatch>> clockMapMap_;

 public:
  JPCCMetric(const JPCCMetricParameter& parameter);

  template <typename PointT>
  void addPoints(const std::string& name, const FramePtr<PointT>& frame);

  template <typename PointT>
  void addPoints(const std::string& name, const GroupOfFrame<PointT>& frames);

  void addBytes(const std::string& name, FrameNumber frameNumber, uint64_t bytes);

  template <typename PointA, typename PointB>
  void addPSNR(const std::string& name, const FramePtr<PointA>& frameA, const FramePtr<PointB>& frameB);

  template <typename PointA, typename PointB>
  void addPSNR(const std::string& name, const GroupOfFrame<PointA>& frameA, const GroupOfFrame<PointB>& frameB);

  template <typename PointA, typename PointB>
  void computePSNR(const FramePtr<PointA>& frameA,
                   const FramePtr<PointB>& frameB,
                   double&                 c2cMSE,
                   double&                 c2cPSNR,
                   double&                 c2pMSE,
                   double&                 c2pPSNR);

  ScopeStopwatch start(const std::string& name, FrameNumber frameNumber);

  void writeAndShow();
};

}  // namespace jpcc::metric

#include <jpcc/metric/impl/JPCCMetric.hpp>
