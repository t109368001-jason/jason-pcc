#pragma once

#include <map>
#include <queue>

#include <pcl/visualization/pcl_visualizer.h>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/visualization/VisualizerParameter.h>

namespace jpcc::visualization {

template <class PointT = Point>
class JPCCVisualizer : public pcl::visualization::PCLVisualizer {
 public:
  using Ptr                = shared_ptr<JPCCVisualizer>;
  using Frame              = jpcc::Frame<PointT>;
  using FramePtr           = typename Frame::Ptr;
  using GroupOfFrame       = jpcc::GroupOfFrame<PointT>;
  using GroupOfFrameMap    = std::map<std::string, GroupOfFrame>;
  using FrameQueue         = std::queue<FramePtr>;
  using RGBColor           = std::array<double, 3>;
  using PointCloudColor    = pcl::visualization::PointCloudColorHandler<PointT>;
  using PointCloudColorPtr = typename PointCloudColor::Ptr;

 protected:
  VisualizerParameter                param_;
  std::string                        name_;
  int                                fontSize_;
  int                                lineHeight_;
  std::string                        primaryId_;
  std::map<std::string, std::string> textMap_;
  std::map<std::string, int>         textHeightMap;
  std::map<std::string, FramePtr>    cloudMap_;
  std::map<std::string, std::string> fieldColorMap_;
  std::map<std::string, RGBColor>    rgbColorMap_;
  mutable std::recursive_mutex       mutex_;
  std::map<std::string, FrameQueue>  queueMap_;

 public:
  JPCCVisualizer(const std::string& name, VisualizerParameter param);

  void updateOrAddText(const std::string& text, int ypos, const std::string& id);

  void updateOrAddCloud(FramePtr cloud, const PointCloudColor& color, const std::string& id);

  void updateText();

  void updateCloud();

  void updateQueue();

  void updateAll();

  void nextFrame();

  void enqueue(const GroupOfFrameMap& map);

  void handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event);

  RGBColor getTextColor(const std::string& id);

  PointCloudColorPtr getCloudColor(const std::string& id, FramePtr cloud);

  void setPrimaryId(const std::string& primaryId);

  void setColor(const std::string& id, const std::string& field);

  void setColor(const std::string& id, double r, double g, double b);

  bool isFull();
};

}  // namespace jpcc::visualization

#include <jpcc/visualization/impl/JPCCVisualizer.hpp>
