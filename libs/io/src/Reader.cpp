#include <jpcc/io/Reader.h>

#include <stdexcept>

#include <jpcc/io/PcapReader.h>
#include <jpcc/io/LvxReader.h>
#include <jpcc/io/PlyReader.h>

using namespace std;

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
typename DatasetReader::Ptr newReader(const DatasetReaderParameter& param, const DatasetParameter& datasetParam) {
  if (datasetParam.type == Type::PCAP) {
    return jpcc::make_shared<PcapReader>(param, datasetParam);
  } else if (datasetParam.type == Type::LVX) {
    return jpcc::make_shared<LvxReader>(param, datasetParam);
  } else if (datasetParam.type >= Type::PLY && datasetParam.type < Type::PLY_END) {
    return jpcc::make_shared<PlyReader>(param, datasetParam);
  } else {
    BOOST_THROW_EXCEPTION(logic_error("Not Implemented "));
  }
}

}  // namespace jpcc::io