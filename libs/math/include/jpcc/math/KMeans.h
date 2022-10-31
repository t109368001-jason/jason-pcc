#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/math/Cluster.h>

namespace jpcc::math {

void kmeans(const std::vector<int32_t>&        samples,
            int                                K,
            std::vector<float>&                centroids,
            std::vector<std::vector<int32_t>>& clusters);

}  // namespace jpcc::math