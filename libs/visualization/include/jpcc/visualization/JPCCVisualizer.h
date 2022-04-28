#pragma once

#include <map>
#include <queue>

#include <pcl/visualization/pcl_visualizer.h>

#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/visualization/JPCCVisualizerBase.h>

namespace jpcc::visualization {

template <typename PointT = Point>
class JPCCVisualizer : public JPCCVisualizerBase {
 public:
  using Ptr                = shared_ptr<JPCCVisualizer>;
  using Frame              = jpcc::Frame<PointT>;
  using FramePtr           = typename Frame::Ptr;
  using GroupOfFrame       = jpcc::GroupOfFrame<PointT>;
  using GroupOfFrameMap    = std::map<std::string, GroupOfFrame>;
  using FrameQueue         = std::queue<FramePtr>;
  using PointCloudColor    = pcl::visualization::PointCloudColorHandler<PointT>;
  using PointCloudColorPtr = typename PointCloudColor::Ptr;

 protected:
  std::map<std::string, FramePtr>   frameMap_;
  std::map<std::string, FrameQueue> queueMap_;

 public:
  JPCCVisualizer(VisualizerParameter param);

  void updateOrAddCloud(FramePtr cloud, const PointCloudColor& color, const std::string& id);

  int updateText(int* windowSize) override;

  void updateCloud();

  void updateQueue();

  void updateAll() override;

  void nextFrame() override;

  void enqueue(const GroupOfFrameMap& framesMap);

  PointCloudColorPtr getCloudColor(const std::string& id, FramePtr cloud);

  bool isFull();
};

}  // namespace jpcc::visualization

#include <jpcc/visualization/impl/JPCCVisualizer.hpp>
