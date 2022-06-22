#pragma once

#include <string>

#include <Eigen/Core>

#include <pcl/point_types.h>

#include <jpcc/common/Common.h>

namespace jpcc::process {

class Condition {
 public:
  enum ConditionType { X, Y, Z, R, REFLECTIVITY, PROD, OR, AND };
  enum ConditionOperation { NONE, GT, GE, LT, LE, EQ };

  shared_ptr<Eigen::Vector4f> coefficient;
  ConditionType               type;
  ConditionOperation          operation;
  double                      threshold;
  std::vector<Condition>      conditions;

  Condition();

  Condition(const std::string& condition);

  Condition(const ConditionType& type, const std::vector<std::string>& conditions);

  template <typename PointT>
  [[nodiscard]] bool predict(const PointT& point) const;

 protected:
  [[nodiscard]] bool predictVector3fMap(pcl::Vector3fMapConst& vector3fMap) const;

  [[nodiscard]] bool predictVector4fMap(pcl::Vector4fMapConst& vector4fMap) const;

  [[nodiscard]] bool predictValue(double val) const;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/Condition.hpp>
