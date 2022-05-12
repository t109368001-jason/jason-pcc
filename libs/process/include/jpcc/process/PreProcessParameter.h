#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>
#include <jpcc/process/RadiusOutlierRemovalParameter.h>
#include <jpcc/process/StatisticalOutlierRemovalParameter.h>
#include <jpcc/process/JPCCConditionalRemovalParameter.h>

namespace jpcc::process {

class PreProcessParameter : public virtual Parameter {
 protected:
  std::string order_;

 public:
  std::vector<std::string>           order;
  RadiusOutlierRemovalParameter      radiusOutlierRemoval;
  StatisticalOutlierRemovalParameter statisticalOutlierRemoval;
  JPCCConditionalRemovalParameter    jpccConditionalRemovalParameter;

  PreProcessParameter();

  PreProcessParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const PreProcessParameter& obj);
};

}  // namespace jpcc::process
