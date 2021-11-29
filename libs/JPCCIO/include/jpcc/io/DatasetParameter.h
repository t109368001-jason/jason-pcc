#ifndef JPCC_COMMON_DATASET_PARAMETER_H_
#define JPCC_COMMON_DATASET_PARAMETER_H_

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace io {

namespace po = boost::program_options;

class DatasetParameter : public jpcc::common::Parameter {
 public:
  std::string              type;
  std::string              folder;
  size_t                   totalFiles;
  std::vector<std::string> files;
  std::vector<size_t>      frameCounts;

  DatasetParameter();

  std::vector<std::array<std::string, 2>> getDependencies() override;

  void check() const;

  friend std::ostream& operator<<(std::ostream& out, const DatasetParameter& obj);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_COMMON_DATASET_PARAMETER_H_