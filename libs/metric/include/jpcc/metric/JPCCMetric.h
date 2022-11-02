#pragma once

#include <map>
#include <set>
#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/common/JPCCContext.h>
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

  void addPoints(const std::string& name, const FramePtr& frame, bool addBytes);

  void addPoints(const std::string& name, const GroupOfFrame& frames, bool addBytes);

  void addPointsAndBytes(const std::string& name, const FramePtr& frame, const std::vector<char>& encodedBytes);

  void addPointsAndBytes(const std::string&                    name,
                         const GroupOfFrame&                   frames,
                         const std::vector<std::vector<char>>& encodedFramesBytes);

  void addBytes(const std::string& name, FrameNumber frameNumber, uint64_t bytes);

  void addBytes(const std::string&                    name,
                FrameNumber                           firstFrameNumber,
                const std::vector<std::vector<char>>& bytesVector);

  void addPSNR(const std::string& name, const FramePtr& frameA, const FramePtr& frameB);

  void addPSNR(const std::string& name, const GroupOfFrame& framesA, const GroupOfFrame& framesB, const bool parallel);

  static void copyNormalToReconstruct(const FramePtr& frame, const FramePtr& reconstructFrame);

  void copyNormalToReconstruct(const GroupOfFrame& frames, const GroupOfFrame& reconstructFrames, bool parallel);

  void copyNormalToReconstruct(const JPCCContext& context, bool parallel);

  void computePSNR(const FramePtr& frameA,
                   const FramePtr& frameB,
                   double&         c2cMSE,
                   double&         c2cPSNR,
                   double&         c2pMSE,
                   double&         c2pPSNR) const;

  ScopeStopwatch start(const std::string& name, FrameNumber frameNumber);

  void writeAndShow();
};

}  // namespace jpcc::metric
