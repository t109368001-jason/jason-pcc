#pragma once

#include <iostream>
#include <string>

#include <jpcc/common/Parameter.h>

namespace jpcc::visualization {

#define VISUALIZER_OPT_PREFIX "visualizer"

class VisualizerParameter : public virtual Parameter {
 protected:
  std::string cameraPosition_;

 public:
  std::array<double, 9> cameraPosition{};
  size_t                bufferSize;

  VisualizerParameter();

  VisualizerParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const VisualizerParameter& obj);
};

}  // namespace jpcc::visualization
