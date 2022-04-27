#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::process {

#define JPCC_NORMAL_ESTIMATION_OPT_PREFIX "jpccNormalEstimation"

class JPCCNormalEstimationParameter : public virtual Parameter {
 public:
  int radiusSearch;
  int kSearch;

  JPCCNormalEstimationParameter();

  JPCCNormalEstimationParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCNormalEstimationParameter& obj);
};

}  // namespace jpcc::process
