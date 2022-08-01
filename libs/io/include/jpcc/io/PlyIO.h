#pragma once

#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/io/DatasetParameter.h>

namespace jpcc::io {

template <typename PointT>
void loadPly(GroupOfFrame<PointT>& frames,
             const std::string&    filePath,
             size_t                startFrameNumber,
             size_t                endFrameNumber,
             bool                  parallel = false);

template <typename PointT>
void savePly(const GroupOfFrame<PointT>& frames, const std::string& filePath, bool parallel = false);

template <typename PointIn, typename PointOut>
void savePly(const GroupOfFrame<PointIn>& frames, const std::string& filePath, bool parallel = false);

}  // namespace jpcc::io

#include <jpcc/io/impl/PlyIO.hpp>
