#ifndef JPCC_IO_DATASET_PARAMETER_H_
#define JPCC_IO_DATASET_PARAMETER_H_

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace io {

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

  std::vector<std::array<std::string, 2>> getDependencies() override;

  void notify() override;

  std::string getFilePath(const size_t index) const;

  friend std::ostream& operator<<(std::ostream& out, const DatasetParameter& obj);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_DATASET_PARAMETER_H_