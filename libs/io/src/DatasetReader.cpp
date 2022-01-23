#include <jpcc/io/DatasetReader.h>

#include <jpcc/io/PcapReader.h>
#include <jpcc/io/LvxReader.h>

namespace jpcc::io {

using namespace std;

DatasetReader::Ptr newReader(const DatasetReaderParameter& param, const DatasetParameter& datasetParam) {
  if (datasetParam.type == "pcap") {
    return DatasetReader::Ptr(new PcapReader(param, datasetParam));
  } else if (datasetParam.type == "lvx") {
    return DatasetReader::Ptr(new LvxReader(param, datasetParam));
  } else {
    BOOST_THROW_EXCEPTION(logic_error(string("Not Implemented ")));
  }
}

}  // namespace jpcc::io