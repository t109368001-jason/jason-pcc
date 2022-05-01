#pragma once

#include <array>
#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/process/PreProcessParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool                               parallel;
  double                             resolution;
  std::string outputCSVPath;
  io::DatasetParameter               dataset;
  io::DatasetReaderParameter         reader;
  process::PreProcessParameter       preProcess;

  AppParameter();

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);
};

}  // namespace jpcc
