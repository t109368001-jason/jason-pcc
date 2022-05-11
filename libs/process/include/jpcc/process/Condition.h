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

  template <typename PointT>
  bool predict(const PointT& point) const;

  bool predictValue(double val) const;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/Condition.hpp>
