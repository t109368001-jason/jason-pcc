#ifndef JPCC_IO_LVX_READER_PARAMETER_H_
#define JPCC_IO_LVX_READER_PARAMETER_H_

#include <iostream>

#include <jpcc/io/ReaderParameterBase.h>

namespace jpcc {
namespace io {

namespace po = boost::program_options;

class LvxReaderParameter : public jpcc::io::ReaderParameterBase {
 public:
  float frequency;
  float interval;  // ms

  LvxReaderParameter();

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const LvxReaderParameter& obj);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_LVX_READER_PARAMETER_H_