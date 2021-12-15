#ifndef JPCC_IO_PCAP_READER_PARAMETER_H_
#define JPCC_IO_PCAP_READER_PARAMETER_H_

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc {
namespace io {

namespace po = boost::program_options;

class PcapReaderParameter : public jpcc::common::Parameter {
 protected:
  std::vector<std::string> pointTypes_;

 public:
  std::set<std::string> pointTypes;
  float                 epsilon;
  bool                  useRadian;

  PcapReaderParameter();

  po::options_description getOpts() override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const PcapReaderParameter& obj);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_PCAP_READER_PARAMETER_H_