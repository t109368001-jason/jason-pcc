#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <jpcc/common/Parameter.h>

namespace jpcc::process {

#define RADIUS_OUTLIER_REMOVAL_OPT_PREFIX "radiusOutlierRemoval"

class RadiusOutlierRemovalParameter : public virtual Parameter {
 public:
  bool  enable;
  float radius;
  int   minNeighborsInRadius;

  RadiusOutlierRemovalParameter();

  RadiusOutlierRemovalParameter(const std::string& prefix, const std::string& caption);

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  friend std::ostream& operator<<(std::ostream& out, const RadiusOutlierRemovalParameter& obj);
};

}  // namespace jpcc::process
