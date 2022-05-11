#pragma once

#include <string>

#include <Eigen/Dense>

#include <jpcc/common/Common.h>
#include <pcl/impl/point_types.hpp>

namespace jpcc::process {

class Condition {
 public:
  enum ConditionType { X, Y, Z, R, PROD };
  enum ConditionOperation { GT, GE, LT, LE, EQ };

  shared_ptr<Eigen::Vector4f> coefficient;
  ConditionType               type;
  ConditionOperation          operation;
  double                      threshold;

  Condition();

  Condition(const std::string& condition);

  template <typename PointT>
  [[nodiscard]] bool predict(const PointT& point) const;

 protected:
  [[nodiscard]] bool predictVector3fMap(pcl::Vector3fMapConst& vector3fMap) const;

  [[nodiscard]] bool predictVector4fMap(pcl::Vector4fMapConst& vector4fMap) const;

  [[nodiscard]] bool predictValue(double val) const;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/Condition.hpp>
