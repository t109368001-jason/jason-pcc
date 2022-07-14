#include <jpcc/visualization/JPCCVisualizer.h>

#include <algorithm>
#include <string>

using namespace std;

namespace jpcc::visualization {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCVisualizer::JPCCVisualizer(const VisualizerParameter& param) : JPCCVisualizerBase(param) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizer::updateOrAddCloud(const FramePtr& cloud, const PointCloudColor& color, const std::string& id) {
  if (!updatePointCloud(cloud, color, id)) {
    addPointCloud(cloud, color, id);
    setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, param_.pointSize, id);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
int JPCCVisualizer::updateText(int* windowSize) {
  int textHeight = JPCCVisualizerBase::updateText(windowSize);
  if (textHeight <= 0) {
    if (!windowSize) { windowSize = getRenderWindow()->GetSize(); }
    textHeight = windowSize[1] - lineHeight_;
  }

  size_t lines = ((frameMap_.find(primaryId_) != frameMap_.end()) ? 2 : 0) + frameMap_.size();
  lines++;  // default fps line

  textHeight = std::min<int>(textHeight, lineHeight_ * lines);
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
  return textHeight;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizer::updateCloud() {
  for (const auto& [id, cloud] : frameMap_) {
    const PointCloudColorPtr color = getCloudColor(id, cloud);
    const RGBColor&          tc    = getTextColor(id);
    updateOrAddCloud(cloud, *color, id);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizer::updateQueue() {
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
void JPCCVisualizer::updateAll() {
  JPCCVisualizerBase::updateAll();
  updateCloud();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizer::nextFrame() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  const auto& it = queueMap_.find(primaryId_);
  if (it == queueMap_.end()) { return; }

  if (param_.outputScreenshot && param_.outputScreenshotBeforeNextFrame) { saveScreenshot(); }

  for (auto& [id, queue] : queueMap_) {
    if (queue.empty()) { continue; }
    frameMap_[id] = queue.front();
    queue.pop();
  }
  updateAll();

  if (param_.outputScreenshot && !param_.outputScreenshotBeforeNextFrame) { saveScreenshot(); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizer::handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event) {
  JPCCVisualizerBase::handleKeyboardEvent(event);
  if (event.keyDown()) {
    switch (event.getKeyCode()) {
      case 'h':
      case 'H':
        std::cout << "\n"
                     "------- JPCCVisualizer\n"
                     "    SHIFT + j, J : save screenshot\n"
                     "    SHIFT + a, A : toggle auto next frame\n"
                     "\n";
        break;
      case 'j':
      case 'J':
        if (event.isShiftPressed() && !event.isCtrlPressed() && !event.isAltPressed()) { saveScreenshot(); }
        break;
      case 'a':
      case 'A':
        if (event.isShiftPressed() && !event.isCtrlPressed() && !event.isAltPressed()) {
          const std::string callbackId = "RenderEvent_autoNextFrame";
          auto              it         = vtkCallbacks_.find(callbackId);
          if (it == vtkCallbacks_.end()) {
            vtkSmartPointer<vtkCallbackCommand> renderCallback = vtkSmartPointer<vtkCallbackCommand>::New();

            renderCallback->SetCallback([](vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* clientData,
                                           void* vtkNotUsed(callData)) {  //
              static_cast<JPCCVisualizer*>(clientData)->nextFrame();
            });
            renderCallback->SetClientData(this);

            getRenderWindow()->AddObserver(vtkCommand::RenderEvent, renderCallback);
            vtkCallbacks_.insert_or_assign(callbackId, renderCallback);
          } else {
            getRenderWindow()->RemoveObserver(it->second);
            vtkCallbacks_.erase(it);
          }
        }
        break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizer::enqueue(const GroupOfFrameMap& framesMap) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  for (const auto& [id, frames] : framesMap) {
    if (queueMap_.find(id) == queueMap_.end()) { queueMap_[id] = FrameQueue(); }
    FrameQueue& queue = queueMap_.at(id);
    for (const FramePtr& frame : frames) { queue.push(frame); }
  }
  updateQueue();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCVisualizer::saveScreenshot() {
  const auto& it = frameMap_.find(primaryId_);
  if (it != frameMap_.end()) {
    spinOnce(1, true);
    const FramePtr& cloud = it->second;

    char fileName[4096] = {0};
    sprintf(fileName, "%s%u.png", param_.outputScreenshotDir.c_str(), cloud->header.seq);
    JPCCVisualizerBase::saveScreenshot(fileName);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
typename JPCCVisualizer::PointCloudColorPtr JPCCVisualizer::getCloudColor(const std::string& id,
                                                                          const FramePtr&    cloud) {
  if (fieldColorMap_.find(id) != fieldColorMap_.end()) {
    return jpcc::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<PointXYZINormal>>(
        cloud, fieldColorMap_.at(id));
  } else if (rgbColorMap_.find(id) != rgbColorMap_.end()) {
    return jpcc::make_shared<pcl::visualization::PointCloudColorHandlerCustom<PointXYZINormal>>(
        cloud, rgbColorMap_.at(id).at(0) * 255.0, rgbColorMap_.at(id).at(1) * 255.0, rgbColorMap_.at(id).at(2) * 255.0);
  } else {
    return jpcc::make_shared<pcl::visualization::PointCloudColorHandlerCustom<PointXYZINormal>>(cloud, 255.0, 255.0,
                                                                                                255.0);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCVisualizer::isFull() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return queueMap_.at(primaryId_).size() >= param_.bufferSize;
}

}  // namespace jpcc::visualization