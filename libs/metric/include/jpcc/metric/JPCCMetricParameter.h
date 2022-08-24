#pragma once

#include <filesystem>
#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::metric {

#define JPCC_METRIC_OPT_PREFIX "jpccMetricParameter"

class JPCCMetricParameter : public virtual Parameter {
 protected:
  std::string outputCSVFolder;

 public:
  std::filesystem::path outputCSVFolderPath;
  double                maximumValue;

  JPCCMetricParameter();

  JPCCMetricParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCMetricParameter& obj);
};

}  // namespace jpcc::metric
