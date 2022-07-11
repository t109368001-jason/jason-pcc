#pragma once

#include <map>
#include <queue>
#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/visualization/JPCCVisualizerBase.h>

namespace jpcc::visualization {

class JPCCVisualizer : public JPCCVisualizerBase {
 public:
  using Ptr                = shared_ptr<JPCCVisualizer>;
  using FrameQueue         = std::queue<FramePtr>;
  using PointCloudColor    = pcl::visualization::PointCloudColorHandler<PointXYZINormal>;
  using PointCloudColorPtr = typename PointCloudColor::Ptr;

 protected:
  std::map<std::string, FramePtr>   frameMap_;
  std::map<std::string, FrameQueue> queueMap_;

 public:
  JPCCVisualizer(const VisualizerParameter& param);

  void updateOrAddCloud(const FramePtr& cloud, const PointCloudColor& color, const std::string& id);

  int updateText(int* windowSize) override;

  void updateCloud();

  void updateQueue();

  void updateAll() override;

  void nextFrame() override;

  void enqueue(const GroupOfFrameMap& framesMap);

  [[nodiscard]] PointCloudColorPtr getCloudColor(const std::string& id, const FramePtr& cloud);

  [[nodiscard]] bool isFull();
};

}  // namespace jpcc::visualization
