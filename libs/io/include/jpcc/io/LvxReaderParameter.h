#ifndef JPCC_IO_LVX_READER_PARAMETER_H_
#define JPCC_IO_LVX_READER_PARAMETER_H_

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace io {

namespace po = boost::program_options;

class LvxReaderParameter : public jpcc::common::Parameter {
 protected:
  std::vector<std::string> pointTypes_;

 public:
  std::set<std::string> pointTypes;
  float                 epsilon;
  bool                  useRadian;
  float                 frequency;
  float                 interval;  // ms

  LvxReaderParameter();

  po::options_description getOpts() override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const LvxReaderParameter& obj);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_LVX_READER_PARAMETER_H_