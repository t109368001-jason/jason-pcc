#pragma once

#include <array>
#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/process/PreProcessParameter.h>
#include <jpcc/process/JPCCNormalEstimationParameter.h>
#include <jpcc/visualization/VisualizerParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool                                     parallel;
  bool                                     previewOnly;
  bool                                     forceReRun;
  double                                   resolution;
  io::DatasetParameter                     dataset;
  io::DatasetReaderParameter               reader;
  process::JPCCConditionalRemovalParameter background;
  process::JPCCConditionalRemovalParameter dynamic;
  visualization::VisualizerParameter       visualizerParameter;

  AppParameter();

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);
};

}  // namespace jpcc
