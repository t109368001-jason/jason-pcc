#pragma once

#include <map>
#include <string>

#include <jpcc/common/Common.h>

namespace jpcc::metric {

class JPCCMetric {
 public:
  using Stopwatch = pcc::chrono::Stopwatch<std::chrono::steady_clock>;

 protected:
  std::map<std::string, Stopwatch> clockMap_;
  std::map<std::string, uint64_t>  pointsMap_;
  std::map<std::string, uint64_t>  bytesMap_;

 public:
  JPCCMetric();

  void start(const std::string& name);

  void stop(const std::string& name);

  template <typename PointT>
  void add(const std::string& name, const FramePtr<PointT>& frame);

  template <typename PointT>
  void add(const std::string& name, const GroupOfFrame<PointT>& frames);

  void addBytes(const std::string& name, uint64_t bytes);

  void show() const;
};

}  // namespace jpcc::metric

#include <jpcc/metric/impl/JPCCMetric.hpp>
