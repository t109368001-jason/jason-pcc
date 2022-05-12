#pragma once

#include <array>
#include <iostream>
#include <string>
#include <map>

#include <jpcc/common/Parameter.h>

namespace jpcc::visualization {

#define VISUALIZER_OPT_PREFIX "visualizer"

using RGBColor = std::array<double, 3>;

class VisualizerParameter : public virtual Parameter {
 protected:
  std::string              cameraPosition_;
  std::vector<std::string> idColors_;

 public:
  std::string                        name;
  std::string                        description;
  bool                               showParameter;
  std::array<double, 9>              cameraPosition;
  size_t                             bufferSize;
  std::map<std::string, std::string> fieldColorMap;
  std::map<std::string, RGBColor>    rgbColorMap;

  VisualizerParameter();

  VisualizerParameter(const std::string& prefix, const std::string& caption);

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const VisualizerParameter& obj);
};

}  // namespace jpcc::visualization
