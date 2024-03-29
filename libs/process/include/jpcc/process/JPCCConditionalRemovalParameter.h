#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>
#include <jpcc/process/Condition.h>

namespace jpcc::process {

#define JPCC_CONDITIONAL_REMOVAL_OPT_PREFIX "jpccConditionalRemoval"

class JPCCConditionalRemovalParameter : public virtual Parameter {
 public:
 protected:
  std::string              type_;
  std::vector<std::string> conditions_;

 public:
  bool                     enable;
  Condition::ConditionType type;
  Condition                condition;

  JPCCConditionalRemovalParameter();

  JPCCConditionalRemovalParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const JPCCConditionalRemovalParameter& obj);
};

}  // namespace jpcc::process
