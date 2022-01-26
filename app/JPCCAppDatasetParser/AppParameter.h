#pragma once

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool                       parallel;
  io::DatasetParameter       inputDatasetParameter;
  io::DatasetReaderParameter inputDatasetReaderParameter;
  io::DatasetParameter       outputDatasetParameter;

  AppParameter();

  void notify() override;
};

}  // namespace jpcc
