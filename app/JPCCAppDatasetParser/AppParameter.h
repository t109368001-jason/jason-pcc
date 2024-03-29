#pragma once

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/process/PreProcessParameter.h>
#include <jpcc/process/JPCCNormalEstimationParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool                                   parallel;
  size_t                                 groupOfFramesSize;
  std::string                            outputPointType;
  io::DatasetParameter                   inputDataset;
  io::DatasetReaderParameter             inputReader;
  io::DatasetParameter                   outputDataset;
  process::PreProcessParameter           preProcess;
  process::JPCCNormalEstimationParameter jpccNormalEstimation;

  AppParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);
};

}  // namespace jpcc
