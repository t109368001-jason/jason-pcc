#pragma once

#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/io/DatasetStreamReader.h>

namespace jpcc::io {

[[nodiscard]] typename DatasetReader::Ptr newReader(const DatasetReaderParameter& param,
                                                    const DatasetParameter&       datasetParam);

}  // namespace jpcc::io
