#pragma once

#include <jpcc/io/DatasetStreamReader.h>

namespace jpcc::io {

template <typename PointT = Point>
using DatasetReader = DatasetStreamReader<PointT>;

template <typename PointT = Point>
using DatasetReaderPtr = typename DatasetReader<PointT>::Ptr;

template <typename PointT = Point>
DatasetReaderPtr<PointT> newReader(const DatasetReaderParameter& param, const DatasetParameter& datasetParam);

}  // namespace jpcc::io

#include <jpcc/io/impl/Reader.hpp>
