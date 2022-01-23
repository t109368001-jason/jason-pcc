#pragma once

#include <jpcc/io/DatasetReaderBase.h>

namespace jpcc::io {

using DatasetReader = DatasetReaderBase;

DatasetReader::Ptr newReader(const DatasetReaderParameter& param, const DatasetParameter& datasetParam);

}  // namespace jpcc::io
