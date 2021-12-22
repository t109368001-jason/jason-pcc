#ifndef JPCC_IO_READER_PARAMETER_BASE_H_
#define JPCC_IO_READER_PARAMETER_BASE_H_

#include <iostream>
#include <set>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace io {

namespace po = boost::program_options;

class ReaderParameterBase : public jpcc::common::Parameter {
 protected:
  std::vector<std::string> pointTypes_;

 public:
  std::set<std::string> pointTypes;
  float                 epsilon;

  ReaderParameterBase(std::string prefix, std::string caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const ReaderParameterBase& obj);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_READER_PARAMETER_BASE_H_