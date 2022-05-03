#pragma once

#include <jpcc/io/DatasetStreamReader.h>

namespace jpcc::io {

template <typename PointT = Point>
typename DatasetReader<PointT>::Ptr newReader(const DatasetReaderParameter& param,
                                              const DatasetParameter&       datasetParam);

}  // namespace jpcc::io

#include <jpcc/io/impl/Reader.hpp>
