#include <stdexcept>

#include <jpcc/io/PcapReader.h>
#include <jpcc/io/LvxReader.h>
#include <jpcc/io/PlyReader.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename DatasetReader<PointT>::Ptr newReader(const DatasetReaderParameter& param,
                                              const DatasetParameter&       datasetParam) {
  if (datasetParam.type == Type::PCAP) {
    return jpcc::make_shared<PcapReader<PointT>>(param, datasetParam);
  } else if (datasetParam.type == Type::LVX) {
    return jpcc::make_shared<LvxReader<PointT>>(param, datasetParam);
  } else if (datasetParam.type >= Type::PLY && datasetParam.type < Type::PLY_END) {
    return jpcc::make_shared<PlyReader<PointT>>(param, datasetParam);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("Not Implemented "));
  }
}

}  // namespace jpcc::io