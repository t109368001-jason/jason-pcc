#include <jpcc/visualization/VisualizerParameter.h>

#include <boost/algorithm/string.hpp>

namespace jpcc::visualization {

using namespace std;
using namespace po;

#define CAMERA_POSITION_OPT_PREFIX ".cameraPosition"
#define BUFFER_SIZE_OPT_PREFIX ".bufferSize"

VisualizerParameter::VisualizerParameter() : VisualizerParameter(VISUALIZER_OPT_PREFIX, __FUNCTION__) {}

VisualizerParameter::VisualizerParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption), cameraPosition({50.0, 0.0, 200.0, 50.0, 0.0, 0.0, 1.0, 0.0, 0.0}), bufferSize(32) {
  cameraPosition.fill(0.0);
  opts_.add_options()                                                    //
      (string(prefix_ + CAMERA_POSITION_OPT_PREFIX).c_str(),             //
       value<string>(&cameraPosition_)->default_value(cameraPosition_),  //
       "cameraPosition")                                                 //
      (string(prefix_ + BUFFER_SIZE_OPT_PREFIX).c_str(),                 //
       value<size_t>(&bufferSize)->default_value(bufferSize),            //
       "bufferSize")                                                     //
      ;
}

void VisualizerParameter::notify() {
  vector<string> ss;
  boost::algorithm::split(ss, cameraPosition_, boost::is_any_of(","));
  assert(ss.size() == cameraPosition.size());
  transform(ss.begin(), ss.end(), cameraPosition.begin(), [](auto&& s) { return stod(s); });
  assert(bufferSize > 0);
}

ostream& operator<<(ostream& out, const VisualizerParameter& obj) {
  out << obj.caption_ << endl;
  out << "\t" << obj.prefix_ << CAMERA_POSITION_OPT_PREFIX "=" << obj.cameraPosition_ << endl;
  out << "\t" << obj.prefix_ << BUFFER_SIZE_OPT_PREFIX "=" << obj.bufferSize << endl;
  return out;
}

}  // namespace jpcc::visualization