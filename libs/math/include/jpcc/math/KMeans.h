#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/math/Cluster.h>

namespace jpcc::math {

void kmeans(const std::vector<Intensity>&        samples,
            int                                  K,
            std::vector<float>&                  centroids,
            std::vector<std::vector<Intensity>>& clusters);

}  // namespace jpcc::math