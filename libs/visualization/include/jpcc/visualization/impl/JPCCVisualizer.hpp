#pragma once

#include <vtkCallbackCommand.h>
#include <vtkNew.h>
#include <vtkObject.h>

namespace jpcc::visualization {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCVisualizer<PointT>::JPCCVisualizer(VisualizerParameter param) :
    PCLVisualizer(param.name),
    param_(std::move(param)),
    fontSize_(16),
    lineHeight_(20),
    primaryId_("cloud"),
    lastWindowHeight_(new int{0}) {
  vtkObject::GlobalWarningDisplayOff();

  initCameraParameters();
  setCameraPosition(param_.cameraPosition.at(0), param_.cameraPosition.at(1), param_.cameraPosition.at(2),
                    param_.cameraPosition.at(3), param_.cameraPosition.at(4), param_.cameraPosition.at(5),
                    param_.cameraPosition.at(6), param_.cameraPosition.at(7), param_.cameraPosition.at(8));
  setBackgroundColor(0, 0, 0);
  addCoordinateSystem(3.0, "coordinate");

  vtkNew<vtkCallbackCommand> modifiedCallback;
  modifiedCallback->SetCallback(
      [](vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* clientData, void* vtkNotUsed(callData)) {
        auto* window     = dynamic_cast<vtkRenderWindow*>(caller);
        int*  windowSize = window->GetSize();
        auto* viewer     = static_cast<JPCCVisualizer<PointT>*>(clientData);
        if (*viewer->lastWindowHeight_ != windowSize[1]) {
          viewer->updateText(windowSize);
          *viewer->lastWindowHeight_ = windowSize[1];
        }
      });
  modifiedCallback->SetClientData(this);

  getRenderWindow()->AddObserver(vtkCommand::ModifiedEvent, modifiedCallback);

  registerKeyboardCallback([this](const auto& event) { this->handleKeyboardEvent(event); });

  registerPointPickingCallback([](const auto& event) {
    Point point;
    event.getPoint(point.x, point.y, point.z);
    cout << "picked point=" << point << endl;
  });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::updateOrAddText(const std::string& text, const int ypos, const std::string& id) {
  const RGBColor& tc = getTextColor(id);
  if (!PCLVisualizer::updateText(text, 5, ypos, fontSize_, tc.at(0), tc.at(1), tc.at(2), id)) {
    addText(text, 5, ypos, fontSize_, tc.at(0), tc.at(1), tc.at(2), id);
  }
  textMap_.insert_or_assign(id, text);
  textHeightMap.insert_or_assign(id, ypos);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::updateOrAddCloud(const FramePtr         cloud,
                                              const PointCloudColor& color,
                                              const std::string&     id) {
  if (!updatePointCloud(cloud, color, id)) { addPointCloud(cloud, color, id); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::updateText(int* windowSize) {
  if (!windowSize) { windowSize = getRenderWindow()->GetSize(); }
  int textHeight = windowSize[1] - lineHeight_;

  if (!param_.description.empty()) {
    updateOrAddText(param_.description, textHeight, "description");
    textHeight -= lineHeight_;
  }
  if ((frameMap_.find(primaryId_) != frameMap_.end())) {
    {
      const FramePtr&   cloud = frameMap_.at(primaryId_);
      const RGBColor&   tc    = getTextColor(primaryId_);
      const std::string id    = primaryId_ + "FrameId";
      const std::string text  = "frame: " + std::to_string(cloud->header.seq);
      setWindowName(param_.name + " " + std::to_string(cloud->header.seq));
      updateOrAddText(text, textHeight, id);
      textHeight -= lineHeight_;
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (queueMap_.find(primaryId_) != queueMap_.end()) {
      const std::string id = primaryId_ + "QueueSize";
      textHeightMap.insert_or_assign(id, textHeight);
      updateQueue();
      textHeight -= lineHeight_;
    }
  }
  for (const auto& [id, cloud] : frameMap_) {
    const PointCloudColorPtr color = getCloudColor(id, cloud);
    const RGBColor&          tc    = getTextColor(id);
    updateOrAddCloud(cloud, *color, id);
    const std::string text = id + " points: " + std::to_string(cloud->size());
    updateOrAddText(text, textHeight, id);
    textHeight -= lineHeight_;
  }
  textHeight = std::min<int>(textHeight, lineHeight_ * parameterTexts_.size());
  if (param_.showParameter) {
    for (size_t i = 0; i < parameterTexts_.size(); i++) {
      updateOrAddText(parameterTexts_.at(i), textHeight, "parameter" + std::to_string(i));
      textHeight -= lineHeight_;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::updateCloud() {
  for (const auto& [id, cloud] : frameMap_) {
    const PointCloudColorPtr color = getCloudColor(id, cloud);
    const RGBColor&          tc    = getTextColor(id);
    updateOrAddCloud(cloud, *color, id);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::updateQueue() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (queueMap_.find(primaryId_) != queueMap_.end()) {
    const FrameQueue& queue      = queueMap_.at(primaryId_);
    const std::string id         = primaryId_ + "QueueSize";
    const std::string text       = "queue: " + std::to_string(queue.size());
    const int         textHeight = textHeightMap[id];
    updateOrAddText(text, textHeight, id);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::updateAll() {
  updateText();
  updateCloud();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::nextFrame() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  for (auto& [id, queue] : queueMap_) {
    if (queue.empty()) { continue; }
    frameMap_[id] = queue.front();
    queue.pop();
  }
  updateAll();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::enqueue(const JPCCVisualizer::GroupOfFrameMap& framesMap) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  for (const auto& [id, frames] : framesMap) {
    if (queueMap_.find(id) == queueMap_.end()) { queueMap_[id] = FrameQueue(); }
    FrameQueue& queue = queueMap_.at(id);
    for (const FramePtr& frame : frames) { queue.push(frame); }
  }
  updateQueue();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::registerKeyboardEvent(KeyboardEvent callback, const std::string& id) {
  keyboardCallbacks_.insert_or_assign(id, callback);
}

// "| Help:\n"
// "-------\n"
//"          p, P   : switch to a point-based representation\n"
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
// "          l, L           : list all available geometric and color handlers for the current actor
// map\n" "    ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)\n"
// "          0..9 [+ CTRL]  : switch between different color handlers (where available)\n"
// "    SHIFT + left click   : select a point (start with -use_point_picking)\n"
// "          x, X   : toggle rubber band selection mode for left mouse button\n"
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event) {
  if (event.keyDown()) {
    switch (event.getKeyCode()) {
      case 'h':
      case 'H':
        std::cout << "\n"
                     "------- JPCCVisualizer\n"
                     "          space  : next frame\n";
        break;
      case ' ': nextFrame(); break;
    }
  }
  for (auto it = keyboardCallbacks_.begin(); it != keyboardCallbacks_.end(); it++) { it->second(event); }
  std::cout << "KeyboardEvent: keyCode=" << event.getKeyCode() << " keyDown=" << event.keyDown() << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename JPCCVisualizer<PointT>::RGBColor JPCCVisualizer<PointT>::getTextColor(const std::string& id) {
  if (rgbColorMap_.find(id) != rgbColorMap_.end()) { return rgbColorMap_.at(id); }
  return {1.0, 1.0, 1.0};
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename JPCCVisualizer<PointT>::PointCloudColorPtr JPCCVisualizer<PointT>::getCloudColor(const std::string& id,
                                                                                          const FramePtr     cloud) {
  if (fieldColorMap_.find(id) != fieldColorMap_.end()) {
    return jpcc::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<PointT>>(cloud,
                                                                                             fieldColorMap_.at(id));
  } else if (rgbColorMap_.find(id) != rgbColorMap_.end()) {
    return jpcc::make_shared<pcl::visualization::PointCloudColorHandlerCustom<PointT>>(
        cloud, rgbColorMap_.at(id).at(0) * 255.0, rgbColorMap_.at(id).at(1) * 255.0, rgbColorMap_.at(id).at(2) * 255.0);
  } else {
    return jpcc::make_shared<pcl::visualization::PointCloudColorHandlerCustom<PointT>>(cloud, 255.0, 255.0, 255.0);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::setPrimaryId(const std::string& primaryId) {
  primaryId_ = primaryId;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::setColor(const std::string& id, const std::string& field) {
  fieldColorMap_[id] = field;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::setColor(const std::string& id, const double r, const double g, const double b) {
  rgbColorMap_[id] = {r, g, b};
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCVisualizer<PointT>::isFull() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return queueMap_.at(primaryId_).size() >= param_.bufferSize;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCVisualizer<PointT>::addParameter(const Parameter& parameter) {
  parameter.getShowTexts(parameterTexts_);
}

}  // namespace jpcc::visualization