#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <jpcc/common/Common.h>

namespace jpcc {

class ParameterOStream {
 protected:
  const std::string prefix_;
  std::ostream&     out_;

 public:
  ParameterOStream(std::ostream& out, std::string prefix);

  template <typename T>
  ParameterOStream& operator()(const std::string& optName, T value);

 protected:
  template <typename T>
  void operator()(std::vector<T> value);

  void operator()(bool value);

  void operator()(const std::vector<shared_ptr<Eigen::Matrix4f>>& matrixVector);

  void operator()(const shared_ptr<Eigen::Matrix4f>& matrix);

  template <typename T>
  void operator()(T value);
};

}  // namespace jpcc

#include <jpcc/common/impl/ParameterOStream.hpp>
