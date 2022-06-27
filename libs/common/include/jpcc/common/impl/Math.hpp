#pragma once

namespace jpcc {

template <class T>
double standard_deviation(const std::vector<T>& values) {
  if constexpr (std::is_same_v<T, int>) {
    const Eigen::VectorXi vector =
        Eigen::Map<const Eigen::VectorXi, Eigen::Unaligned>(values.data(), (int)values.size());
    return std::sqrt((vector.array() - vector.mean()).square().sum() / (double)vector.size());
  } else if constexpr (std::is_same_v<T, float>) {
    const Eigen::VectorXf vector =
        Eigen::Map<const Eigen::VectorXf, Eigen::Unaligned>(values.data(), (int)values.size());
    return std::sqrt((vector.array() - vector.mean()).square().sum() / (double)vector.size());
  } else if constexpr (std::is_same_v<T, double>) {
    const Eigen::VectorXd vector =
        Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(values.data(), (int)values.size());
    return std::sqrt((vector.array() - vector.mean()).square().sum() / (double)vector.size());
  }
}

}  // namespace jpcc