#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <jpcc/common/Common.h>

namespace jpcc {

class Analyzer {
 public:
  using Ptr           = shared_ptr<Analyzer>;
  using Frame         = jpcc::Frame<PointNormal>;
  using FrameConstPtr = typename Frame::ConstPtr;

 protected:
  std::filesystem::path filepath_;

 public:
  Analyzer(const std::string& outputDir, const std::string& filename);

  [[nodiscard]] const std::filesystem::path& getFilepath() const;

  [[nodiscard]] bool exists();

  virtual void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) = 0;

  virtual void finalCompute() = 0;
};

}  // namespace jpcc