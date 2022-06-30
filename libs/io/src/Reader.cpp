#include <jpcc/io/Reader.h>

#include <stdexcept>

#include <jpcc/io/PcapReader.h>
#include <jpcc/io/LvxReader.h>
#include <jpcc/io/PlyReader.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
typename DatasetReader::Ptr newReader(const DatasetReaderParameter& param, const DatasetParameter& datasetParam) {
  if (datasetParam.type == "pcap") {
    return jpcc::make_shared<PcapReader>(param, datasetParam);
  } else if (datasetParam.type == "lvx") {
    return jpcc::make_shared<LvxReader>(param, datasetParam);
  } else if (datasetParam.type == "ply") {
    return jpcc::make_shared<PlyReader>(param, datasetParam);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error(std::string("Not Implemented ")));
  }
}

}  // namespace jpcc::io