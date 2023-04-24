#pragma once

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool                       parallel;
  size_t                     groupOfFramesSize;
  io::DatasetParameter       dataset;
  io::DatasetReaderParameter reader;

  AppParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);
};

}  // namespace jpcc
