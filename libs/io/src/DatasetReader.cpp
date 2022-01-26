#include <jpcc/io/DatasetReader.h>

#include <jpcc/io/PcapReader.h>
#include <jpcc/io/LvxReader.h>
#include <jpcc/io/PlyReader.h>

namespace jpcc::io {

using namespace std;

DatasetReader::Ptr newReader(const DatasetReaderParameter& param, const DatasetParameter& datasetParam) {
  if (datasetParam.type == "pcap") {
    return DatasetReader::Ptr(new PcapReader(param, datasetParam));
  } else if (datasetParam.type == "lvx") {
    return DatasetReader::Ptr(new LvxReader(param, datasetParam));
  } else if (datasetParam.type == "ply") {
    return DatasetReader::Ptr(new PlyReader(param, datasetParam));
  } else {
    BOOST_THROW_EXCEPTION(logic_error(string("Not Implemented ")));
  }
}

}  // namespace jpcc::io