#pragma once

#include <jpcc/io/DatasetReaderBase.h>

namespace jpcc::io {

template <typename PointT = Point>
using DatasetReader = DatasetReaderBase<PointT>;

template <typename PointT = Point>
using DatasetReaderPtr = typename DatasetReaderBase<PointT>::Ptr;

template <typename PointT = Point>
DatasetReaderPtr<PointT> newReader(const DatasetReaderParameter& param, const DatasetParameter& datasetParam);

}  // namespace jpcc::io

#include <jpcc/io/impl/Reader.hpp>
