#pragma once

#include <jpcc/io/PcapReader.h>
#include <jpcc/io/LvxReader.h>
#include <jpcc/io/PlyReader.h>

namespace jpcc::io {

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
DatasetReaderPtr<PointT> newReader(const DatasetReaderParameter& param, const DatasetParameter& datasetParam) {
  if (datasetParam.type == "pcap") {
    return DatasetReaderPtr<PointT>(new PcapReader<PointT>(param, datasetParam));
  } else if (datasetParam.type == "lvx") {
    return DatasetReaderPtr<PointT>(new LvxReader<PointT>(param, datasetParam));
  } else if (datasetParam.type == "ply") {
    return DatasetReaderPtr<PointT>(new PlyReader<PointT>(param, datasetParam));
  } else {
    BOOST_THROW_EXCEPTION(logic_error(string("Not Implemented ")));
  }
}

}  // namespace jpcc::io