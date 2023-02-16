#include <jpcc/visualization/JPCCVisualizerBase.h>

#include <boost/log/trivial.hpp>

#include <vtkObject.h>

namespace jpcc::visualization {

JPCCVisualizerBase::JPCCVisualizerBase(const VisualizerParameter& param) :
    PCLVisualizer(param.name),
    param_(param),
    fontSize_(16),
    lineHeight_(20),
    primaryId_("cloud"),
    fieldColorMap_(param_.fieldColorMap),
    rgbColorMap_(param_.rgbColorMap),
    lastWindowHeight_(new int{0}) {
  vtkObject::GlobalWarningDisplayOff();

  initCameraParameters();
  setCameraPosition(param_.cameraPosition[0], param_.cameraPosition[1], param_.cameraPosition[2],
                    param_.cameraPosition[3], param_.cameraPosition[4], param_.cameraPosition[5],
                    param_.cameraPosition[6], param_.cameraPosition[7], param_.cameraPosition[8]);
  setBackgroundColor(0, 0, 0);
  addCoordinateSystem(1000.0, "coordinate");

  vtkSmartPointer<vtkCallbackCommand> modifiedCallback = vtkSmartPointer<vtkCallbackCommand>::New();
  modifiedCallback->SetCallback(
      [](vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* clientData, void* vtkNotUsed(callData)) {
        auto* window     = dynamic_cast<vtkRenderWindow*>(caller);
        int*  windowSize = window->GetSize();
        auto* viewer     = static_cast<JPCCVisualizerBase*>(clientData);

        if (*viewer->lastWindowHeight_ != windowSize[1]) {
          viewer->updateText(windowSize);
          *viewer->lastWindowHeight_ = windowSize[1];
        }
      });
  modifiedCallback->SetClientData(this);

  getRenderWindow()->AddObserver(vtkCommand::ModifiedEvent, modifiedCallback);
  vtkCallbacks_.insert_or_assign("ModifiedEvent_updateText", modifiedCallback);

  registerKeyboardCallback([this](const auto& event) { this->handleKeyboardEvent(event); });

  registerPointPickingCallback([](const auto& event) {
    pcl::PointXYZ point;
    event.getPoint(point.x, point.y, point.z);
    BOOST_LOG_TRIVIAL(info) << "picked point=" << point;
  });
  if (param_.windowWidth != 0 && param_.windowHeight != 0) { setSize(param_.windowWidth, param_.windowHeight); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizerBase::updateOrAddText(const std::string& text, const int ypos, const std::string& id) {
  const RGBColor& tc = getTextColor(id);
  if (!PCLVisualizer::updateText(text, 5, ypos, fontSize_, tc[0], tc[1], tc[2], id)) {
    addText(text, 5, ypos, fontSize_, tc[0], tc[1], tc[2], id);
  }
  textMap_.insert_or_assign(id, text);
  textHeightMap.insert_or_assign(id, ypos);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int JPCCVisualizerBase::updateText(int* windowSize) {
  if (!windowSize) { windowSize = getRenderWindow()->GetSize(); }
  int textHeight = windowSize[1] - lineHeight_;

  if (!param_.description.empty()) {
    updateOrAddText(param_.description, textHeight, "description");
    textHeight -= lineHeight_;
  }
  if (param_.showParameter) {
    for (size_t i = 0; i < parameterTexts_.size(); i++) {
      updateOrAddText(parameterTexts_[i], textHeight, "parameter" + std::to_string(i));
      textHeight -= lineHeight_;
    }
  } else {
    for (size_t i = 0; i < parameterTexts_.size(); i++) {
      updateOrAddText("", textHeight, "parameter" + std::to_string(i));
    }
  }
  return textHeight;
}

void JPCCVisualizerBase::updateAll() { updateText(nullptr); }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizerBase::registerKeyboardEvent(const KeyboardEvent& callback, const std::string& id) {
  keyboardCallbacks_.insert_or_assign(id, callback);
}

// "| Help:\n"
// "-------\n"
// "          p, P   : switch to a point-based representation\n"
// "          w, W   : switch to a wireframe-based representation (where available)\n"
// "          s, S   : switch to a surface-based representation (where available)\n"
// "          j, J   : take a .PNG snapshot of the current window view\n"
// "          c, C   : display current camera/window parameters\n"
// "          f, F   : fly to point mode\n"
// "          e, E   : exit the interactor\n"
// "          q, Q   : stop and call VTK's TerminateApp\n"
// "           +/-   : increment/decrement overall point size\n"
// "     +/- [+ ALT] : zoom in/out \n"
// "          g, G   : display scale grid (on/off)\n"
// "          u, U   : display lookup table (on/off)\n"
// "    o, O         : switch between perspective/parallel projection (default = perspective)\n"
// "    r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]\n"
// "    CTRL + s, S  : save camera parameters\n"
// "    CTRL + r, R  : restore camera parameters\n"
// "    ALT + s, S   : turn stereo mode on/off\n"
// "    ALT + f, F   : switch between maximized window mode and original size\n"
// "          l, L           : list all available geometric and color handlers for the current actor  map\n"
// "    ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)\n"
// "          0..9 [+ CTRL]  : switch between different color handlers (where available)\n"
// "    SHIFT + left click   : select a point (start with -use_point_picking)\n"
// "          x, X   : toggle rubber band selection mode for left mouse button\n"
//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizerBase::handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event) {
  if (event.keyDown()) {
    switch (event.getKeyCode()) {
      case 'h':
      case 'H':
        BOOST_LOG_TRIVIAL(info) << "\n"
                                   "------- JPCCVisualizerBase\n"
                                   "         space   : next frame\n"
                                   "    SHIFT + p, P : toggle display parameters\n";
        break;
      case ' ': nextFrame(); break;
      case 'p':
      case 'P':
        if (event.isShiftPressed() && !event.isCtrlPressed() && !event.isAltPressed()) {
          param_.showParameter = !param_.showParameter;
          updateText(nullptr);
        }
        break;
    }
  }
  for (auto& keyboardCallback : keyboardCallbacks_) { keyboardCallback.second(event); }
  if (event.getKeyCode() != 0) {
    BOOST_LOG_TRIVIAL(info) << "KeyboardEvent: keyCode=" << event.getKeyCode() << " keyDown=" << event.keyDown();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
RGBColor JPCCVisualizerBase::getTextColor(const std::string& id) {
  if (rgbColorMap_.find(id) != rgbColorMap_.end()) { return rgbColorMap_[id]; }
  return {1.0, 1.0, 1.0};
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizerBase::setPrimaryId(const std::string& primaryId) { primaryId_ = primaryId; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizerBase::setColor(const std::string& id, const std::string& field) {
  const auto& it = rgbColorMap_.find(id);
  if (it != rgbColorMap_.end()) { rgbColorMap_.erase(it); }
  fieldColorMap_[id] = field;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizerBase::setColor(const std::string& id, const double r, const double g, const double b) {
  const auto& it = fieldColorMap_.find(id);
  if (it != fieldColorMap_.end()) { fieldColorMap_.erase(it); }
  rgbColorMap_[id] = {r, g, b};
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizerBase::addParameter(const Parameter& parameter) { parameter.getShowTexts(parameterTexts_); }

}  // namespace jpcc::visualization