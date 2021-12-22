#ifndef JPCC_IO_PCAP_READER_PARAMETER_H_
#define JPCC_IO_PCAP_READER_PARAMETER_H_

#include <jpcc/io/ReaderParameterBase.h>

namespace jpcc {
namespace io {

class PcapReaderParameter : public jpcc::io::ReaderParameterBase {
 public:
  PcapReaderParameter();
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_PCAP_READER_PARAMETER_H_