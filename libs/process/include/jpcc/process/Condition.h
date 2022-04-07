#pragma once

#include <string>

namespace jpcc::process {

class Condition {
 public:
  enum ConditionField { X, Y, Z, R };
  enum ConditionOperation { GT, GE, LT, LE, EQ };

  ConditionField     field;
  ConditionOperation operation;
  double             threshold;

  Condition();

  Condition(const std::string& condition);
};

}  // namespace jpcc::process