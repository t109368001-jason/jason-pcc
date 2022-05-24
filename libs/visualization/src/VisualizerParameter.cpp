#include <jpcc/visualization/VisualizerParameter.h>

#include <boost/algorithm/string.hpp>

namespace jpcc::visualization {

using namespace std;
using namespace po;

#define NAME_OPT ".name"
#define DESCRIPTION_OPT ".description"
#define SHOW_PARAMETER_OPT ".showParameter"
#define CAMERA_POSITION_OPT ".cameraPosition"
#define BUFFER_SIZE_OPT ".bufferSize"
#define POINT_SIZE_OPT ".pointSize"
#define WINDOW_WIDTH_OPT ".windowWidth"
#define WINDOW_HEIGHT_OPT ".windowHeight"
#define ID_COLORS_OPT ".idColors"

VisualizerParameter::VisualizerParameter() : VisualizerParameter(VISUALIZER_OPT_PREFIX, __FUNCTION__) {}

VisualizerParameter::VisualizerParameter(const string& prefix, const string& caption) :
    Parameter(prefix, caption),
    name("JPCC Visualizer"),
    description(),
    showParameter(true),
    cameraPosition({50.0, 0.0, 200.0, 50.0, 0.0, 0.0, 1.0, 0.0, 0.0}),
    bufferSize(32),
    pointSize(1),
    windowWidth(0),
    windowHeight(0) {
  cameraPosition.fill(0.0);
  opts_.add_options()                                                    //
      (string(prefix_ + NAME_OPT).c_str(),                               //
       value<string>(&name)->default_value(name),                        //
       "name")                                                           //
      (string(prefix_ + DESCRIPTION_OPT).c_str(),                        //
       value<string>(&description)->default_value(description),          //
       "description")                                                    //
      (string(prefix_ + SHOW_PARAMETER_OPT).c_str(),                     //
       value<bool>(&showParameter)->default_value(showParameter),        //
       "showParameter")                                                  //
      (string(prefix_ + CAMERA_POSITION_OPT).c_str(),                    //
       value<string>(&cameraPosition_)->default_value(cameraPosition_),  //
       "cameraPosition")                                                 //
      (string(prefix_ + BUFFER_SIZE_OPT).c_str(),                        //
       value<size_t>(&bufferSize)->default_value(bufferSize),            //
       "bufferSize")                                                     //
      (string(prefix_ + POINT_SIZE_OPT).c_str(),                         //
       value<size_t>(&pointSize)->default_value(pointSize),              //
       "pointSize")                                                      //
      (string(prefix_ + WINDOW_WIDTH_OPT).c_str(),                       //
       value<int>(&windowWidth)->default_value(windowWidth),             //
       "windowWidth")                                                    //
      (string(prefix_ + WINDOW_HEIGHT_OPT).c_str(),                      //
       value<int>(&windowHeight)->default_value(windowHeight),           //
       "windowHeight")                                                   //
      (string(prefix_ + ID_COLORS_OPT).c_str(),                          //
       value<vector<string>>(&idColors_),                                //
       "idColors")                                                       //
      ;
}

void VisualizerParameter::notify() {
  {
    vector<string> ss;
    boost::algorithm::split(ss, cameraPosition_, boost::is_any_of(","));
    assert(ss.size() == cameraPosition.size());
    transform(ss.begin(), ss.end(), cameraPosition.begin(), [](auto&& s) { return stod(s); });
  }
  assert(bufferSize > 0);
  assert(pointSize > 0);
  for (const string& idColor : idColors_) {
    vector<string> ss;
    boost::algorithm::split(ss, idColor, boost::is_any_of(","));
    if (ss.size() == 2) {  // field color
      string field = ss.at(1);
      boost::trim(field);
      assert(field == "x" || field == "y" || field == "z");
      fieldColorMap.insert_or_assign(ss.at(0), field);
    } else if (ss.size() == 4) {  // rgb color
      double r = stod(ss.at(1));
      assert(r >= 0.0);
      double g = stod(ss.at(2));
      assert(g >= 0.0);
      double b = stod(ss.at(3));
      assert(b >= 0.0);
      if (r > 1.0 || g > 1.0 || b > 1.0) {
        r /= 255.0;
        g /= 255.0;
        b /= 255.0;
      }
      rgbColorMap.insert_or_assign(ss.at(0), RGBColor{r, g, b});
    }
  }
}

ostream& operator<<(ostream& out, const VisualizerParameter& obj) {
  obj.coutParameters(out)                         //
      (NAME_OPT, obj.name)                        //
      (DESCRIPTION_OPT, obj.description)          //
      (SHOW_PARAMETER_OPT, obj.showParameter)     //
      (CAMERA_POSITION_OPT, obj.cameraPosition_)  //
      (BUFFER_SIZE_OPT, obj.bufferSize)           //
      (POINT_SIZE_OPT, obj.pointSize)             //
      (WINDOW_WIDTH_OPT, obj.windowWidth)         //
      (WINDOW_HEIGHT_OPT, obj.windowHeight)       //
      (ID_COLORS_OPT, obj.idColors_)              //
      ;
  return out;
}

}  // namespace jpcc::visualization