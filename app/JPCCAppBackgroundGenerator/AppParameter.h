#pragma once

#include <array>
#include <iostream>
#include <string>

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
  std::string                            outputPath;
  double                                 backgroundThreshold;
  double                                 filterResolution;
  double                                 backgroundResolution;
  io::DatasetParameter                   dataset;
  io::DatasetReaderParameter             reader;
  process::PreProcessParameter           preProcess;
  process::JPCCNormalEstimationParameter jpccNormalEstimation;

  AppParameter();

  void notify() override;

  [[nodiscard]] std::string getOutputPath() const;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);
};

}  // namespace jpcc
