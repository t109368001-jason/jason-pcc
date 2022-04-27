#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::process {

#define STATISTICAL_OUTLIER_REMOVAL_OPT_PREFIX "statisticalOutlierRemoval"

class StatisticalOutlierRemovalParameter : public virtual Parameter {
 public:
  bool   enable;
  size_t meanK;
  float  stddevMulThresh;

  StatisticalOutlierRemovalParameter();

  StatisticalOutlierRemovalParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const StatisticalOutlierRemovalParameter& obj);
};

}  // namespace jpcc::process
