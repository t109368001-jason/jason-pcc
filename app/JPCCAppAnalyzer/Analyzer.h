#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <jpcc/common/Common.h>

namespace jpcc {

class Analyzer {
 public:
  using Ptr = shared_ptr<Analyzer>;

 protected:
  float                 frequency_;
  double                resolution_;
  std::filesystem::path filepath_;
  bool                  hasLock;

 public:
  Analyzer(const float&       frequency,
           const double&      resolution,
           const std::string& outputDir,
           const std::string& title,
           const std::string& otherParameters = "");

  ~Analyzer();

  [[nodiscard]] const std::filesystem::path& getFilepath() const;

  [[nodiscard]] bool exists();

  [[nodiscard]] bool tryLockFile();

  void releaseLockFile();

  void saveCloud();

  virtual void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) = 0;

  virtual void finalCompute() = 0;

  virtual void getCloud(FramePtr& cloud) = 0;

  virtual void reset();
};

}  // namespace jpcc