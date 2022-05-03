#pragma once

#include <jpcc/io/PcapReader.h>
#include <jpcc/io/LvxReader.h>
#include <jpcc/io/PlyReader.h>

namespace jpcc::io {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename DatasetReader<PointT>::Ptr newReader(const DatasetReaderParameter& param,
                                              const DatasetParameter&       datasetParam) {
  if (datasetParam.type == "pcap") {
    return jpcc::make_shared<PcapReader<PointT>>(param, datasetParam);
  } else if (datasetParam.type == "lvx") {
    return jpcc::make_shared<LvxReader<PointT>>(param, datasetParam);
  } else if (datasetParam.type == "ply") {
    return jpcc::make_shared<PlyReader<PointT>>(param, datasetParam);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error(std::string("Not Implemented ")));
  }
}

}  // namespace jpcc::io