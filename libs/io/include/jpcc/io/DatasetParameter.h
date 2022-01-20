#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc::io {

class DatasetParameter : public jpcc::common::Parameter {
 public:
  std::string              sensor;
  std::string              type;
  std::string              folderPrefix;
  std::string              folder;
  size_t                   totalFiles;
  std::vector<std::string> files;
  std::vector<size_t>      frameCounts;
  bool                     haveGpsTime;

  DatasetParameter();

  [[nodiscard]] std::vector<std::array<std::string, 2>> getDependencies() const override;

  void notify() override;

  [[nodiscard]] std::string getFilePath(size_t index) const;

  friend std::ostream& operator<<(std::ostream& out, const DatasetParameter& obj);
};

}  // namespace jpcc::io
