#include <jpcc/io/PcapReaderParameter.h>

namespace jpcc {
namespace io {

#define PCAP_READER_OPT_PREFIX "pcapReader"

PcapReaderParameter::PcapReaderParameter() : ReaderParameterBase(PCAP_READER_OPT_PREFIX, "PcapReaderParameter") {}

}  // namespace io
}  // namespace jpcc
