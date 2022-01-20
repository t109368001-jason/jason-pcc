#include <jpcc/io/PcapReaderParameter.h>

namespace jpcc::io {

#define PCAP_READER_OPT_PREFIX "pcapReader"

PcapReaderParameter::PcapReaderParameter() : ReaderParameterBase(PCAP_READER_OPT_PREFIX, "PcapReaderParameter") {}

}  // namespace jpcc::io
