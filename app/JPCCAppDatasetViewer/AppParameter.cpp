#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT_PREFIX ".parallel"
#define CAMERA_POSITION_OPT_PREFIX ".cameraPosition"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__), parallel(false), datasetParameter(), readerParameter() {
  cameraPosition_.fill(0.0);
  opts_.add_options()                                                  //
      (string(prefix_ + PARALLEL_OPT_PREFIX).c_str(),                  //
       value<bool>(&parallel)->default_value(parallel),                //
       "parallel")                                                     //
      (string(prefix_ + CAMERA_POSITION_OPT_PREFIX).c_str(),           //
       value<string>(&cameraPosition)->default_value(cameraPosition),  //
       "cameraPosition")                                               //
      ;
}

void AppParameter::notify() {
  vector<string> ss;
  boost::algorithm::split(ss, cameraPosition, boost::is_any_of(","));
  assert(ss.size() == cameraPosition_.size());
  std::transform(ss.begin(), ss.end(), cameraPosition_.begin(), [](auto&& s) { return std::stod(s); });
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << PARALLEL_OPT_PREFIX "=" << obj.parallel << endl;
  out << "\t" << obj.prefix_ << CAMERA_POSITION_OPT_PREFIX "=" << obj.cameraPosition << endl;
  out << obj.datasetParameter;
  out << obj.readerParameter;
  return out;
}

void AppParameter::applyCameraPosition(const std::function<void(double pos_x,
                                                                double pos_y,
                                                                double pos_z,
                                                                double view_x,
                                                                double view_y,
                                                                double view_z,
                                                                double up_x,
                                                                double up_y,
                                                                double up_z)>& func) const {
  func(cameraPosition_[0], cameraPosition_[1], cameraPosition_[2], cameraPosition_[3], cameraPosition_[4],
       cameraPosition_[5], cameraPosition_[6], cameraPosition_[7], cameraPosition_[8]);
}

}  // namespace jpcc