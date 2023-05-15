#pragma once

#include <jpcc/common/Parameter.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>

namespace jpcc {

class AppParameter : public Parameter {
 public:
  bool                       parallel;
  size_t                     groupOfFramesSize;
  int                        qp;
  io::DatasetParameter       inputDataset;
  io::DatasetReaderParameter inputReader;
  io::DatasetParameter       outputDataset;

  AppParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const AppParameter& obj);
};

}  // namespace jpcc
