#pragma once

#include <fstream>
#include <string>
#include <utility>

#include <jpcc/common/Common.h>

namespace jpcc {

class Analyzer {
 public:
  using Ptr           = shared_ptr<Analyzer>;
  using Frame         = Frame<PointNormal>;
  using FrameConstPtr = typename Frame::ConstPtr;

 protected:
  std::string filename_;

 public:
  Analyzer(std::string filename);

  virtual bool exists() = 0;

  virtual void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) = 0;

  virtual void finalCompute() = 0;

  std::string getFilename();
};

}  // namespace jpcc