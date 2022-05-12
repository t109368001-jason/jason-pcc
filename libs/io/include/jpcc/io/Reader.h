#pragma once

#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/io/DatasetStreamReader.h>

namespace jpcc::io {

template <typename PointT = Point>
[[nodiscard]] typename DatasetReader<PointT>::Ptr newReader(const DatasetReaderParameter& param,
                                                            const DatasetParameter&       datasetParam);

}  // namespace jpcc::io

#include <jpcc/io/impl/Reader.hpp>
