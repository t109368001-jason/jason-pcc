#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/math/Cluster.h>

namespace jpcc::math {

void kmeans(const std::vector<float>&        samples,
            int                              K,
            std::vector<float>&              centroids,
            std::vector<std::vector<float>>& clusters);

}  // namespace jpcc::math