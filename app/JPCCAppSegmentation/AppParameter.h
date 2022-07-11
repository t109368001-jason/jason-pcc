#pragma once

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/process/PreProcessParameter.h>
#include <jpcc/process/JPCCNormalEstimationParameter.h>
#include <jpcc/segmentation/JPCCGMMSegmentationParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool                                       parallel;
  io::DatasetParameter                       inputDataset;
  io::DatasetReaderParameter                 inputReader;
  io::DatasetParameter                       outputDataset;
  process::PreProcessParameter               preProcess;
  process::JPCCNormalEstimationParameter     jpccNormalEstimation;
  segmentation::JPCCGMMSegmentationParameter jpccGmmSegmentation;

  AppParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);
};

}  // namespace jpcc