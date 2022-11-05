#pragma once

#include <string>

#include <Eigen/Core>

#include <pcl/point_types.h>

#include <jpcc/common/Common.h>

namespace jpcc::process {

class Condition {
 public:
  enum ConditionType { X, Y, Z, R, PROD, OR, AND };
  enum ConditionOperation { NONE, GT, GE, LT, LE, EQ };

  shared_ptr<Eigen::Vector4f> coefficient;
  ConditionType               type;
  ConditionOperation          operation;
  double                      threshold;
  std::vector<Condition>      conditions;

  Condition();

  Condition(const std::string& condition);  // NOLINT(google-explicit-constructor)

  Condition(const ConditionType& type, const std::vector<std::string>& conditions);

  [[nodiscard]] bool predict(const Frame::PointType& point) const;

 protected:
  [[nodiscard]] bool predictVector3fMap(const Frame::PointType& point) const;

  [[nodiscard]] bool predictValue(double val) const;
};

}  // namespace jpcc::process
