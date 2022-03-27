#include "AppParameter.h"

#include <boost/algorithm/string.hpp>

namespace jpcc {

using namespace std;
using namespace po;

#define APP_OPT_PREFIX "app"
#define PARALLEL_OPT_PREFIX ".parallel"
#define CAMERA_POSITION_OPT_PREFIX ".cameraPosition"

AppParameter::AppParameter() :
    Parameter(APP_OPT_PREFIX, __FUNCTION__), parallel(false), dataset(), reader(), preProcess() {
  cameraPosition.fill(0.0);
  opts_.add_options()                                                    //
      (string(prefix_ + PARALLEL_OPT_PREFIX).c_str(),                    //
       value<bool>(&parallel)->default_value(parallel),                  //
       "parallel")                                                       //
      (string(prefix_ + CAMERA_POSITION_OPT_PREFIX).c_str(),             //
       value<string>(&cameraPosition_)->default_value(cameraPosition_),  //
       "cameraPosition")                                                 //
      ;
  opts_.add(dataset.getOpts());
  opts_.add(reader.getOpts());
  opts_.add(preProcess.getOpts());
}

void AppParameter::notify() {
  vector<string> ss;
  boost::algorithm::split(ss, cameraPosition_, boost::is_any_of(","));
  assert(ss.size() == cameraPosition.size());
  transform(ss.begin(), ss.end(), cameraPosition.begin(), [](auto&& s) { return stod(s); });
  dataset.notify();
  reader.notify();
  preProcess.notify();
}

ostream& operator<<(ostream& out, const AppParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << PARALLEL_OPT_PREFIX "=" << obj.parallel << endl;
  out << "\t" << obj.prefix_ << CAMERA_POSITION_OPT_PREFIX "=" << obj.cameraPosition_ << endl;
  out << obj.dataset;
  out << obj.reader;
  out << obj.preProcess;
  return out;
}

void AppParameter::applyCameraPosition(const function<void(double pos_x,
                                                           double pos_y,
                                                           double pos_z,
                                                           double view_x,
                                                           double view_y,
                                                           double view_z,
                                                           double up_x,
                                                           double up_y,
                                                           double up_z)>& func) const {
  func(cameraPosition[0], cameraPosition[1], cameraPosition[2], cameraPosition[3], cameraPosition[4], cameraPosition[5],
       cameraPosition[6], cameraPosition[7], cameraPosition[8]);
}

}  // namespace jpcc