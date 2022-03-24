#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc::io {

class DatasetParameter : public virtual Parameter {
 public:
  std::string              name;
  std::string              sensor;
  std::string              type;
  std::string              folderPrefix;
  std::string              folder;
  std::vector<std::string> files;
  std::vector<size_t>      frameCounts;
  std::vector<size_t>      startFrameNumbers;
  bool                     haveGpsTime;

  DatasetParameter();

  DatasetParameter(const std::string& prefix, const std::string& caption);

  [[nodiscard]] std::vector<std::array<std::string, 2>> getDependencies() const override;

  void notify() override;

  [[nodiscard]] size_t count() const;

  [[nodiscard]] std::string getFilePath(size_t index = 0) const;

  [[nodiscard]] size_t getFrameCounts(size_t index = 0) const;

  [[nodiscard]] size_t getStartFrameNumbers(size_t index = 0) const;

  friend std::ostream& operator<<(std::ostream& out, const DatasetParameter& obj);
};

}  // namespace jpcc::io
