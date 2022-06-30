#include <jpcc/io/Pcap.h>

#include <exception>
#include <sstream>
#include <stdexcept>

#include <pcap/pcap.h>

#include <jpcc/common/Common.h>

using namespace std;

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
void* pcapOpen(const string& pcapPath) {
  char          error[PCAP_ERRBUF_SIZE];
  pcap_t* const pcap = pcap_open_offline(pcapPath.c_str(), error);
  if (!pcap) { throw runtime_error(error); }

  struct bpf_program filter = {0};
  ostringstream      oss;
  if (pcap_compile(pcap, &filter, oss.str().c_str(), 0, 0xffffffff) == -1) { throw runtime_error(pcap_geterr(pcap)); }

  if (pcap_setfilter(pcap, &filter) == -1) { throw runtime_error(pcap_geterr(pcap)); }
  return static_cast<void*>(pcap);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int pcapNextEx(void* pcap, const unsigned char** data) {
  struct pcap_pkthdr* header;

  const int ret = pcap_next_ex(static_cast<pcap_t*>(pcap), &header, data);
  if (ret <= 0) { return ret; }

  // Check Packet Data Size
  // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
  if ((header->len - 42) != 1206) { return 2; }
  return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcapClose(void* pcap) { pcap_close(static_cast<pcap_t*>(pcap)); }

}  // namespace jpcc::io