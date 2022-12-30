#pragma once

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/metric/JPCCMetricParameter.h>
#include <jpcc/process/JPCCNormalEstimationParameter.h>
#include <jpcc/process/PreProcessParameter.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>
#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool                                    parallel;
  size_t                                  groupOfFramesSize;
  std::string                             compressedStreamPathPrefix;
  io::DatasetParameter                    dataset;
  io::DatasetReaderParameter              reader;
  process::PreProcessParameter            preProcess;
  segmentation::JPCCSegmentationParameter jpccGmmSegmentation;
  encoder::JPCCEncoderParameter           jpccEncoderStatic;
  encoder::JPCCEncoderParameter           jpccEncoderDynamic;
  process::JPCCNormalEstimationParameter  normalEstimation;
  metric::JPCCMetricParameter             metricParameter;

  AppParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);
};

}  // namespace jpcc
