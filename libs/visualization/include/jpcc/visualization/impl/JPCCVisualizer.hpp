#pragma once

#include <vtkCallbackCommand.h>
#include <vtkNew.h>

namespace jpcc::visualization {

using namespace std;
using namespace pcl::visualization;

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
JPCCVisualizer<PointT>::JPCCVisualizer(const string& name, VisualizerParameter param) :
    PCLVisualizer(name), param_(move(param)), name_(name), fontSize_(16), lineHeight_(20), primaryId_("cloud") {
  initCameraParameters();
  setCameraPosition(param_.cameraPosition[0], param_.cameraPosition[1], param_.cameraPosition[2],
                    param_.cameraPosition[3], param_.cameraPosition[4], param_.cameraPosition[5],
                    param_.cameraPosition[6], param_.cameraPosition[7], param_.cameraPosition[8]);
  setBackgroundColor(0, 0, 0);
  addCoordinateSystem(3.0, "coordinate");

  function<void(vtkObject*, long unsigned int, void*, void*)> windowModifiedCallback =
      [&](vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* vtkNotUsed(clientData),
          void* vtkNotUsed(callData)) { updateText(); };

  vtkNew<vtkCallbackCommand> m_pModifiedCallback;
  m_pModifiedCallback->SetCallback(windowModifiedCallback.target<void(vtkObject*, long unsigned int, void*, void*)>());
  getRenderWindow()->AddObserver(vtkCommand::ModifiedEvent, m_pModifiedCallback);

  registerKeyboardCallback([this](auto& event) { this->handleKeyboardEvent(event); });

  registerPointPickingCallback([](auto& event) {
    Point point;
    event.getPoint(point.x, point.y, point.z);
    cout << "picked point=" << point << endl;
  });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::updateOrAddText(const string& text, int ypos, const string& id) {
  const RGBColor& tc = getTextColor(id);
  if (!PCLVisualizer::updateText(text, 5, ypos, fontSize_, tc[0], tc[1], tc[2], id)) {
    addText(text, 5, ypos, fontSize_, tc[0], tc[1], tc[2], id);
  }
  textMap_.insert_or_assign(id, text);
  textHeightMap.insert_or_assign(id, ypos);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::updateOrAddCloud(FramePtr cloud, const PointCloudColor& color, const string& id) {
  if (!updatePointCloud(cloud, color, id)) { addPointCloud(cloud, color, id); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::updateText() {
  int* windowSize = getRenderWindow()->GetSize();
  int  textHeight = windowSize[1] - lineHeight_;

  if ((cloudMap_.find(primaryId_) != cloudMap_.end())) {
    {
      const FramePtr& cloud = cloudMap_.at(primaryId_);
      const RGBColor& tc    = getTextColor(primaryId_);
      const string    id    = primaryId_ + "FrameId";
      const string    text  = "frame: " + to_string(cloud->header.seq);
      setWindowName(name_ + " " + to_string(cloud->header.seq));
      updateOrAddText(text, textHeight, id);
      textHeight -= lineHeight_;
    }
    lock_guard<recursive_mutex> lock(mutex_);
    if (queueMap_.find(primaryId_) != queueMap_.end()) {
      const string id   = primaryId_ + "QueueSize";
      textHeightMap[id] = textHeight;
      updateQueue();
      textHeight -= lineHeight_;
    }
  }
  for (const auto& [id, cloud] : cloudMap_) {
    PointCloudColorPtr color = getCloudColor(id, cloud);
    const RGBColor&    tc    = getTextColor(id);
    updateOrAddCloud(cloud, *color, id);
    const string text = id + " points: " + to_string(cloud->size());
    updateOrAddText(text, textHeight, id);
    textHeight -= lineHeight_;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::updateCloud() {
  for (const auto& [id, cloud] : cloudMap_) {
    PointCloudColorPtr color = getCloudColor(id, cloud);
    const RGBColor&    tc    = getTextColor(id);
    updateOrAddCloud(cloud, *color, id);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::updateQueue() {
  lock_guard<recursive_mutex> lock(mutex_);
  if (queueMap_.find(primaryId_) != queueMap_.end()) {
    const FrameQueue& queue      = queueMap_.at(primaryId_);
    const string      id         = primaryId_ + "QueueSize";
    const string      text       = "queue: " + to_string(queue.size());
    int               textHeight = textHeightMap[id];
    updateOrAddText(text, textHeight, id);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::updateAll() {
  updateText();
  updateCloud();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::nextFrame() {
  lock_guard<recursive_mutex> lock(mutex_);
  for (auto& [id, queue] : queueMap_) {
    if (queue.empty()) { continue; }
    cloudMap_[id] = queue.front();
    queue.pop();
  }
  updateAll();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::enqueue(const JPCCVisualizer::GroupOfFrameMap& map) {
  lock_guard<recursive_mutex> lock(mutex_);
  for (const auto& [id, frames] : map) {
    if (queueMap_.find(id) == queueMap_.end()) { queueMap_[id] = FrameQueue(); }
    FrameQueue& queue = queueMap_.at(id);
    for (FramePtr frame : frames) { queue.push(frame); }
  }
  updateQueue();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event) {
  if (event.getKeyCode() == ' ' && event.keyDown()) { nextFrame(); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
typename JPCCVisualizer<PointT>::RGBColor JPCCVisualizer<PointT>::getTextColor(const string& id) {
  if (rgbColorMap_.find(id) != rgbColorMap_.end()) { return rgbColorMap_.at(id); }
  return {1.0, 1.0, 1.0};
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
typename JPCCVisualizer<PointT>::PointCloudColorPtr JPCCVisualizer<PointT>::getCloudColor(const string& id,
                                                                                          FramePtr      cloud) {
  PointCloudColorPtr color;
  if (fieldColorMap_.find(id) != fieldColorMap_.end()) {
    color.reset(new PointCloudColorHandlerGenericField<PointT>(cloud, fieldColorMap_.at(id)));
  } else if (rgbColorMap_.find(id) != rgbColorMap_.end()) {
    color.reset(new PointCloudColorHandlerCustom<PointT>(cloud, rgbColorMap_.at(id).at(0) * 255.0,
                                                         rgbColorMap_.at(id).at(1) * 255.0,
                                                         rgbColorMap_.at(id).at(2) * 255.0));
  } else {
    color.reset(new PointCloudColorHandlerCustom<PointT>(cloud, 255.0, 255.0, 255.0));
  }
  return color;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::setPrimaryId(const string& primaryId) {
  primaryId_ = primaryId;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::setColor(const string& id, const string& field) {
  fieldColorMap_[id] = field;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
void JPCCVisualizer<PointT>::setColor(const string& id, const double r, const double g, const double b) {
  rgbColorMap_[id] = {r, g, b};
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <class PointT>
bool JPCCVisualizer<PointT>::isFull() {
  lock_guard<recursive_mutex> lock(mutex_);
  return queueMap_.at(primaryId_).size() >= param_.bufferSize;
}

}  // namespace jpcc::visualization