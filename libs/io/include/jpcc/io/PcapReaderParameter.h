#ifndef JPCC_IO_PCAP_READER_PARAMETER_H_
#define JPCC_IO_PCAP_READER_PARAMETER_H_

#include <jpcc/io/ReaderParameterBase.h>

namespace jpcc::io {

class PcapReaderParameter : public jpcc::io::ReaderParameterBase {
 public:
  PcapReaderParameter();
};

}  // namespace jpcc::io

#endif  // JPCC_IO_PCAP_READER_PARAMETER_H_